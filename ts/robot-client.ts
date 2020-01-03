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


const kDebugCallbacks = true;

// These will be loaded from the config JSON file.
// =========================================
let imageCompression = 40;
var topics : RobotChannelDescription[] = [
  { name: "/visualization", 
    type: "f1tenth_course/VisualizationMsg",
    maxRate: 10
  }
];
// =========================================

const rosnodejs = require('rosnodejs');
// const f1tenth_msgs = rosnodejs.require('f1tenth_course').msg;
console.log("Loading packages...");
rosnodejs.loadAllPackages();
console.log("Done.");


function spinSubscribers(rosNode: any) {
  topics.forEach(function(topic) {
    console.log(`Subscribing to ${topic.name} of type ${topic.type} at max rate of ${topic.maxRate}Hz`); 
    subscribe(rosNode, topic.name, topic.type, 1000 / topic.maxRate);
  });
}

function sendUpdate(msg: RobotChannel) {
}

function createGenericCallback(name: string, type: string) {
  return function(data: any) {
    if (kDebugCallbacks) {
      console.log(`Received msg of type ${type} from ${name}`);
    }
    sendUpdate({
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
        // console.log(`image: ${imageBuffer.length}`);
        // console.log(`length: ${msgJson.length}`);
        sendUpdate({
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
    sendUpdate({
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

    if (typeof(req.body.rostopic) !== 'string'||
        typeof(req.body.rostype) !== 'string' ||
        typeof(req.body.rosmsg) !== 'string') {
      console.error(`Malformed sendRobot request from ${req.ip}:`)
      console.log(req.body);
      res.status(400).send("Malformed sendRobot request");
      return;
    }

    let rostopic = req.body.rostopic;
    let rostype = req.body.rostype;
    let rosmsg = JSON.parse(req.body.rosmsg);

    //console.log(`Server callback: ${req}`);
    let response = {
      error: 0,
      data: "Hello world"
    }
    res.status(200).send(JSON.stringify(response));
  } catch(e) {
    console.log(e.toString);
    res.status(500).send("Exception: " + e.toString);
  }
}

function main(rosNode: any) {
  spinSubscribers(rosNode);
}

rosnodejs.initNode('/deployment_client',{ onTheFly: true }).then(main);

console.log("Internal IP: " + internalIP.v4.sync());

export const callbackServer = express();

callbackServer.use(function(req, res, next) {
  res.header("Access-Control-Allow-Origin", "*");
  res.header("Access-Control-Allow-Headers", "Origin, X-Requested-With, Content-Type, Accept");
  next();
});
callbackServer.use(bodyParser.json({ limit: '10mb' }));
callbackServer.use(bodyParser.urlencoded({ extended: true, limit: '10mb' }));
callbackServer.use(compression());
callbackServer.post('/Callback', serverCallback);
callbackServer.listen(9143, () => console.log('RoboServer Callback listening on port 9143'))
