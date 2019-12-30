import * as http from 'http'
import { RobotChannel, RobotChannelDescription } from './types'
import * as sharp from 'sharp'
import * as fs from 'fs'
import { Request, Response } from 'express';
import * as compression from 'compression'
import * as express from 'express';
import * as bodyParser from 'body-parser'
import * as RoboTypes from './types'
import * as https from 'https'
import * as internalIP from 'internal-ip'


// Settings.
// =========================================
let imageCompression = 40;
let robotChannelData = [] as RobotChannel[];
// =========================================

const kDebugCallbacks = false;

const rosnodejs = require('rosnodejs');
console.log("Loading packages...");
rosnodejs.loadAllPackages();
console.log("Done.");

let allowedRosTopics = ['/orb_nav/goal', '/task_execution/task'];
let allowedRosTypes = ['jackal_msgs_amrl/NavGoal', 'task_assignment/TaskAssignmentAction'];

function UpdateChannel(data: RobotChannel) {
  for (let i = 0; i < robotChannelData.length; ++i) {
    if (robotChannelData[i].name === data.name &&
        robotChannelData[i].type === data.type) {
      robotChannelData[i] = data;
      return;
    }
  }
  robotChannelData.push(data);
}

function spinSubscribers(rosNode: any, topics: RobotChannelDescription[]) {
  topics.forEach(function(topic) {
    console.log(`Subscribing to ${topic.name} of type ${topic.type} at max rate of ${topic.maxRate}Hz`); 
    subscribe(rosNode, topic.name, topic.type, 1000 / topic.maxRate);
  });
}

function createGenericCallback(name: string, type: string) {
  return function(data: any) {
    if (kDebugCallbacks) {
      console.log(`Received msg of type ${type} from ${name}`);
    }
    UpdateChannel({
      name: name,
      type: type,
      data: data
    });
  };
}

function image16To8(imageData: number[],
                    channels: 1 | 3 | 4,
                    inputBits: number,
                    isBigEndian: boolean) {
  const packing = 2 * channels;
  const upperMask = (inputBits > 8) ?
    ((1 << (inputBits - 8)) - 1) : 
    0;
  const totalMask = (1 << inputBits) - 1;
  for (let i = 0; i < imageData.length; i += 2) {
    let b0 = isBigEndian? imageData[i + 1] : imageData[i];
    let b1 = isBigEndian? imageData[i] : imageData[i + 1];
    const upper = ((imageData[i+1] & upperMask) << 8);
    const lower = imageData[i];
    imageData[i / packing] = ((lower + upper) & totalMask);
  }
}

function createImageCallback(name: string, type: string) {
  return function(msg: any) {
    if (kDebugCallbacks) {
      console.log(`Received msg of type ${type} from ${name}`);
    }
    let numChannels = 0;
    let imageData = msg.data;
    switch(msg.encoding) {
      case 'mono8': {
        numChannels = 1;
      } break;
      case 'rgb8': {
        numChannels = 3;
      } break;
      case 'rgba8': {
        numChannels = 4;
      } break;
      case 'mono16': {
        image16To8(imageData, 1, 12, msg.is_bigendian !== 0);
        numChannels = 1;
      } break;
      default: {
        console.error(`Unexpected image encoding ${msg.encoding}`);
        return;
      }
    }
    const encoding = { raw: {
      width: msg.width,
      height: msg.height,
      channels: numChannels
    }};
    const image = sharp(imageData, encoding)
      .resize(320, undefined, {
        kernel: 'nearest'
      });
    image.jpeg({ quality: imageCompression })
      .toBuffer()
      .then(function(imageBuffer) {
        if (kDebugCallbacks) {
          console.log(`Image size: ${imageBuffer.length}`);
        }
        msg.data = imageBuffer.toString('base64');
        const msgJson = JSON.stringify(msg);
        console.log(`image: ${imageBuffer.length}`);
        console.log(`length: ${msgJson.length}`);
        UpdateChannel({
          name: name,
          type: type,
          data: msgJson
        });
    });
  };
}

function createCompressedImageCallback(name: string, type: string) {
  return function(msg: any) {
    if (kDebugCallbacks) {
      console.log(`Received msg of type ${type} from ${name}`);
    }
    msg.data = msg.data.toString('base64');
    // console.log(msg.data);
    // process.exit(0);
    const msgJson = JSON.stringify(msg);
    console.log(`Compressed image of data ${msgJson.length}`);
    UpdateChannel({
      name: name,
      type: type,
      data: msgJson
    });
  };
}

function createCallback(name: string, type: string) {
  if (type === 'sensor_msgs/Image') {
    return createImageCallback(name, type);
  } else if (type === 'sensor_msgs/CompressedImage') {
    return createCompressedImageCallback(name, type);
  } else {
    return createGenericCallback(name, type);
  }
}

function subscribe(rosNode: any,
                   topic: string,
                   type: string,
                   throttle: number) {
  return rosNode.subscribe(
    topic,
    type,
    createCallback(topic, type),
    {queueSize: 1, throttleMs: throttle});
}

async function serverCallback(req: Request, res: Response) {
  try {

    if (typeof(req.body.rostopic) !== 'string' ||
        allowedRosTopics.indexOf(req.body.rostopic) === -1 ||
        typeof(req.body.rostype) !== 'string' ||
        allowedRosTypes.indexOf(req.body.rostype) === -1 ||
        typeof(req.body.rosmsg) !== 'string') {
      console.error(`Malformed sendRobot request from ${req.ip}:`)
      console.log(req.body);
      res.status(400).send("Malformed sendRobot request");
      return;
    }

    let rostopic = req.body.rostopic;
    let rostype = req.body.rostype;
    let rosmsg = JSON.parse(req.body.rosmsg);

    //TODO: Publish a ROS topic with this info

    const response = {
      time: new Date(),
      response: 'Ok'
    };
    //console.log(`Server callback: ${req}`);
    res.status(200).send(JSON.stringify(response));
  } catch(e) {
    console.log(e.toString);
    res.status(500).send("Exception: " + e.toString);
  }
}

function main(rosNode: any) {
  let topics = [
    {name: '/scan', type: 'sensor_msgs/LaserScan', maxRate: 10},
    {name: '/test', type: 'std_msgs/String', maxRate: 10}
  ]; 
  spinSubscribers(rosNode, topics);
}

rosnodejs.initNode('/web_client',{ onTheFly: false }).then(main);

console.log("Internal IP: " + internalIP.v4.sync());


function getChannelData(channelName: string) : RoboTypes.RobotChannelType | undefined {
  const channel = robotChannelData.find((c) => { return c.name === channelName;});
  if (channel === undefined) {
    // TODO: Return an error code?
    console.log(`ERROR: robot channel ${channelName} not found.`)
    return undefined;
  }
  return channel.data;
}

function staticRoot(req: Request, res: Response) {
  res.sendfile('', {root: 'static/live-view.html'});
}

async function getRobotStatus(req: Request, res: Response) {
  if (typeof(req.body.channel) !== 'string') {
    console.error(`Malformed robot update from ${req.ip}:`)
    res.status(400).send("Malformed robot update request");
    return;
  }
  const channel = req.body.channel;
  const channelData = getChannelData(channel);
  if (channelData === undefined) {
    res.status(400).send('ERROR: Unknown channel');  
  } else {
    res.status(200).send(channelData);
  }
}

async function sendCmd(req: Request, res: Response) {
  const nonce = 'sudo do this';

  //TODO What type is start/end_time

  if (req.body.data === undefined ||
      typeof(req.body.auth) !== 'string') {
    console.error(`Malformed sendRobotCmd request from ${req.ip}:`)
    res.status(400).send("Malformed sendRobotCmd request");
    return;
  }
  if (req.body.auth !== nonce) {
    console.error(`Unauthorized sendRobotCmd request from ${req.ip}:`)
    res.status(403).send("Not authorized");
    return;
  }
}

export const webServer = express();
webServer.use(function(req, res, next) {
  res.header("Access-Control-Allow-Origin", "*");
  res.header("Access-Control-Allow-Headers", "Origin, X-Requested-With, Content-Type, Accept");
  next();
});
webServer.use(bodyParser.json({ limit: '10mb' }));
webServer.use(bodyParser.urlencoded({ extended: true, limit: '10mb' }));
webServer.use(compression());
webServer.get('/', staticRoot);
webServer.post('/RobotStatus', getRobotStatus);
webServer.post('/SendCmd', sendCmd);

process.on('uncaughtException', function (err) {
  console.log(err);
}); 

// HTTP: Used for robot update
webServer.listen(9043, () => console.log('webServer listening on port 9043'))