import { Request, Response } from 'express'; // For response and request object autocomplete
import * as express from 'express'; // for routing different links
import * as bodyParser from 'body-parser'
import * as RoboTypes from './types'
import * as compression from 'compression'
import * as jpeg from 'jpeg-js'
import * as fs from 'fs'
import * as https from 'https'
const http = require('http');
const spawn = require("child_process").spawn;
http.post = require('http-post');

/*
General documentation: Robot state will be pushed by each robot. Data channels 
from robot are crypotgraphically secured using WireGuard. Thus, wireguard 
ensures that: 
1. Each robot has a unique IP address on the VPN (10.0.*.*).
2. Each IP address is secured by that robot's public key, and hence cannot be
   impersonated.
Thus, the robofleet server just needs to maintain a list of robots based on 
their originating IPs.
*/

// The main robot status store.
export const roboFleetStore: RoboTypes.RobotState[] = [];

type ChannelDescription = {name: string, type: string};
type RobotDescription = {name: string, channels: ChannelDescription[]};

let robotStatus: any = {};
let taskStatus: any = {};
let assignments: any = {};
let nextTaskId = 1;

let robotIpMap = new Map();
robotIpMap.set("UnknownRobot1", "10.0.0.1");
robotIpMap.set("UnknownRobot2", "10.0.0.3");
robotIpMap.set("UnknownRobot3", "10.0.0.5");
robotIpMap.set("Jackal", "10.0.0.101");
robotIpMap.set("UnknownRobot4", "10.0.0.102");
robotIpMap.set("UnknownRobot5", "10.0.0.103");

function channelDescription(c: RoboTypes.RobotChannel) {
  return {name: c.name, type: c.type};
}

function getChannels(r: RoboTypes.RobotState) {
  return {
    name: r.id.name,
    channels: r.channels.reduce((s, r) => {
      s.push(channelDescription(r));
      return s;
    }, [] as ChannelDescription[])
  }
}

async function getServerStatus(req: Request, res: Response) {
  try {
    const response = {
      time: new Date(),
      robots: roboFleetStore.reduce((s, r) => { 
        s.push(getChannels(r)); return s; }, [] as RobotDescription[])
    };
    // console.log(response);
    res.status(200).send(JSON.stringify(response));
  } catch(e) {
    res.status(500).send("Exception: " + e.toString);
  }
}

function updateTopic(msg: RoboTypes.RobotUpdate, ip: string) {
  // console.log(`Update to ${msg.name}`);
  let robot = roboFleetStore.find((r) => { return r.id.name === msg.name; });
  if (robot === undefined) {
    // Add a new robot definition.
    robot = newRobot(msg.name, ip);
    roboFleetStore.push(robot);
  }
  const channel = robot.channels.find((c) => { 
    return c.name === msg.data.name;});
  if (channel === undefined) {
    // Add new channel.
    robot.channels.push(msg.data);
  } else {
    // Update existing channel.
    channel.data = msg.data.data;
  }
  robot.lastUpdated = new Date();
}

function isAuthorizedRobot(ip: string) {
  
  let authorizedRobots = Array.from( robotIpMap.values() );

  // Note: ip matching performed by string sub-match to hackily do IPV6->IPV4 
  // conversion. Security is still maintained by the VPN (see general doc).
  return authorizedRobots.reduce(
    (authorized, robot) => {return authorized || ip.includes(robot);}, false);
}

function getChannelData(robotName: string, channelName: string) : RoboTypes.RobotChannelType {
  let robot = roboFleetStore.find((r) => { return r.id.name === robotName; });
  if (robot === undefined) {
    // TODO: Return an error code?
    return '';
  }
  const channel = robot.channels.find((c) => { return c.name === channelName;});
  if (channel === undefined) {
    // TODO: Return an error code?
    return '';
  }
  return channel.data;
}

async function getData(req: Request, res: Response) {

  if (typeof(req.body.type) !== 'string' || 
      typeof(req.body.subdir) !== 'string' ||
      typeof(req.body.key) !== 'string') {
    console.error(`Malformed robot update from ${req.ip}:`)
    res.status(400).send("Malformed robot update request");
    return;
  }

  const reqType = req.body.type;
  const reqSubDir = req.body.subdir;
  const reqKey = req.body.key;

  let path = "./data/"
  if(reqSubDir === "") {
  	path += reqType + "/" + reqKey;
  } else {
  	path += reqType + "/" + reqSubDir + "/" + reqKey;
  }

  fs.readFile(path, (err1, data1) => {
    if(!err1) {
      res.status(200).send(data1);
    } else {
      path += ".json"
      fs.readFile(path, (err2, data2) => {
        if(!err2) {
          res.status(200).send(data2);
        } else {
          res.status(400).send("File Does Not Exist");
        }
      });
    }
  });

}

async function setData(req: Request, res: Response) {

  if (typeof(req.body.type) !== 'string' || 
      typeof(req.body.subdir) !== 'string' ||
      typeof(req.body.key) !== 'string' ||
      typeof(req.body.data) !== 'string') {
    console.error(`Malformed robot update from ${req.ip}:`)
    res.status(400).send("Malformed robot update request");
    return;
  }

  const reqType = req.body.type;
  const reqSubDir = req.body.subdir;
  const reqFileName = req.body.key;
  const data = req.body.data;

  let path = "./data/"
  if(reqSubDir === "") {
  	path += reqType;
  } else {
  	path += reqType + "/" + reqSubDir;
  }

  fs.exists(path, (exists) => {
    if(!exists) {
      //Path Does Not Exist
      //Attempting to create it and then make the file
      fs.mkdir(path, { recursive: true }, (err) => {
        if(err){
          //Error Creating Path
          res.status(400).send("Error: " + err.code);
        } else {
          //Created the Path, Now making the file
          path += "/" + reqFileName;
          fs.writeFile(path, data, (err) => {
            if(err) {
              res.status(400).send("Error: " + err.code);
            } else {
              res.status(200).send("Success!");
            }
          });
        }
      });
    } else {
      //Path Does Exist, Now Attempting to Make the file
      path += "/" + reqFileName;
      fs.writeFile(path, data, (err) => {
        if(err) {
          res.status(400).send("Error: " + err.code);
        } else {
          res.status(200).send("Success!");
        }
      });
    }
  });
}

async function sendRobotTo(req: Request, res: Response) {
  const nonce = 'sudo do this';
  if (typeof(req.body.robot) !== 'string' || 
      typeof(req.body.building) !== 'string' ||
      typeof(req.body.x) !== 'number' ||
      typeof(req.body.y) !== 'number' ||
      typeof(req.body.auth) !== 'string') {
    console.error(`Malformed sendRobotTo request from ${req.ip}:`)
    res.status(400).send("Malformed sendRobotTo request");
    return;
  }
  if (req.body.auth !== nonce) {
    console.error(`Unauthorized sendRobotTo request from ${req.ip}:`)
    res.status(403).send("Not authorized");
    return;
  }
  try {
    const reqRobot = req.body.robot;
    const reqBuilding = req.body.building;
    const reqX = req.body.x;
    const reqY = req.body.y;

    if (!robotIpMap.has(reqRobot)) {
      console.error(`Invalid Robot ${reqRobot}:`)
      res.status(400).send("Invalid Robot");
      return;
    }

    let rosmsg = {
      x: reqX,
      y: reqY
    }

    let successFunc = function(res: Response){
      res.status(200).send("Ok");
    }

    let failFunc = function(res: Response){
      res.status(400).send("Error sending topic to robot");
    }

    publishTopicToRobot(reqRobot, '/orb_nav/goal', 'jackal_msgs_amrl/NavGoal', rosmsg, successFunc, res, failFunc, res);

  } catch(e) {
    console.error("sendRobotTo Error:" + e);
    res.status(400).send("Unknown error");
    return;
  }

}

async function sendRobotCmd(req: Request, res: Response) {
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
  try {

    let successful = true;
    let failReason = '';

    let data = req.body.data;

    for (let robot in data) {

      if (!data.hasOwnProperty(robot)) continue;

      let task = data[robot];

      if (!robotIpMap.has(robot) ||
          !robotStatus.hasOwnProperty(robot) ||
          !taskStatus.hasOwnProperty(task)) {

        console.error(`Invalid Robot: ${robot} Or Task: ${task}`)
        failReason = 'Task Not Recieved --- Invalid Robot: ${robot} Or Task: ${task}';
        successful = false;
        continue;
      }

      let task_req = {
        id: task,
        type: taskStatus[task]['task_request']['type'],
        start_time: taskStatus[task]['task_request']['start_time'],
        end_time: taskStatus[task]['task_request']['end_time'],
        data: JSON.stringify(taskStatus[task]['task_request']['data'])
      }

      let rosmsg = {
        robot_id: robot,
        task_request: task_req
      }

      assignments[robot] = task;

      let successFunc = function(params: any){
        let task = params[0];
        let robot = params[1];
        taskStatus[task]['status'] = 'inProgress';
        robotStatus[robot]['status'] = 'unavailable';
        //console.log('Robot ${robot} Assigned Task: ${task}');
      }

      let failFunc = function(params: any){
        let task = params[0];
        let robot = params[1];
        console.error(`Error Sending Task ${task} To Robot ${robot}`);
      }

      publishTopicToRobot(robot, '/task_execution/task', 'task_assignment/TaskAssignmentAction', rosmsg, successFunc, [task, robot], failFunc, [task, robot]);
      
    }

    if(successful) {
      res.status(200).send("Ok");
    } else {
      res.status(400).send(failReason);
    }

  } catch(e) {
    console.error("sendRobotCmd Error:" + e);
    res.status(400).send("Unknown error");
    return;
  }

}

async function sendPlannerCmd(req: Request, res: Response) {
  const nonce = 'sudo do this';
  if (typeof(req.body.robot) !== 'string' || 
      typeof(req.body.action) !== 'string' ||
      typeof(req.body.from) !== 'string' ||
      typeof(req.body.to) !== 'string' ||
      typeof(req.body.auth) !== 'string') {
    console.error(`Malformed sendPlannerCmd request from ${req.ip}:`)
    res.status(400).send("Malformed sendPlannerCmd request");
    return;
  }
  if (req.body.auth !== nonce) {
    console.error(`Unauthorized sendPlannerCmd request from ${req.ip}:`)
    res.status(403).send("Not authorized");
    return;
  }
  try {
    const reqRobot = req.body.robot;
    const reqAct = req.body.action;
    const reqFrom = req.body.from;
    const reqTo = req.body.to;

    let taskInfoObj = {
      status: "available",
      task_request: {
        type: reqAct,
        start_time: 0,
        end_time: 100,
        data: {
          package: "default",
          pickup_location: reqFrom,
          dropoff_location: reqTo
        }
      }
    }

    taskStatus[nextTaskId] = taskInfoObj;
    nextTaskId = nextTaskId + 1;

    res.status(200).send("OK");

  } catch(e) {
    console.error("sendPlannerCmd Error:" + e);
    res.status(400).send("Unknown error");
    return;
  }

}

async function publishTopicToRobot(robot: string, rostopic: string, rostype: string, rosmsg: any, successFunction: any, successArgs: any, failFunction: any, failArgs: any) {

    if (!robotIpMap.has(robot)) {
      console.error(`Invalid Robot ${robot}:`)
      return false;
    }

    let robotIp = robotIpMap.get(robot);
    
    //Send POST to this IP Address
    let url = 'http://' + robotIp + ':9143/Callback';

    const query = {
      rostopic: rostopic,
      rostype: rostype,
      rosmsg: JSON.stringify(rosmsg)
    };

    let post_req = http.post(url, query, function(post_res : any){
      post_res.setEncoding('utf8');
      post_res.on('data', function(chunk : any) {
        //Success Case
        //console.log(chunk);
        successFunction(successArgs);
      });
    });


    post_req.on('error', (e : any) => {
      console.error('Error in post to Robot: ' + e.message);
      failFunction(failArgs);
    });

    return post_req;
  
}

async function runRobotAssignment() {

  const path = "./tmp/task_and_robot_status_info.json";

  const jsonObject = {
    tasks: taskStatus,
    robots: robotStatus,
    assignments: assignments
  };

  const jsonObjectString = JSON.stringify(jsonObject);

  let path2Python = "../agents/src/multiagent-sas/task_assignment/src/greedy_task_assignment_node.py";

  fs.writeFile(path, jsonObjectString, (err) => {
    if(err) {
      console.log("Error Writing JSON file: " + err.code);
    } else {
      const pythonProcess = spawn('python',[path2Python]);
      pythonProcess.stdout.on('data', function(data: any){
        console.log(data.toString());
      });
    }
  });

}

async function getRobotStatus(req: Request, res: Response) {
  if (typeof(req.body.name) !== 'string' || 
      typeof(req.body.channel) !== 'string') {
    console.error(`Malformed robot update from ${req.ip}:`)
    res.status(400).send("Malformed robot update request");
    return;
  }
  const robot = req.body.name;
  const channel = req.body.channel;
  const channelData = getChannelData(robot, channel);
  res.status(200).send(channelData);
}

async function updateState(req: Request, res: Response) {
  if (!isAuthorizedRobot(req.ip)) {
    console.error(`Received unauthorized update from ${req.ip}`);
    res.status(403).send("Forbidden");
    return;
  }
  try {
    // console.log(`Update request from ${req.ip}`);
    const msg = req.body;
    if (msg.name !== undefined && msg.name !== undefined) {
      updateTopic(msg as RoboTypes.RobotUpdate, req.ip);
    } else {
      console.error(`Malformed robot update from ${req.ip}: ` + msg);
      res.status(400).send("Malformed robot update request");
      return;
    }
    res.status(200).send("OK");

    if(msg.data.type === 'task_execution/RobotStatus'
        && typeof(msg.data.data.id) === 'string'
        && typeof(msg.data.data.status) === 'string'
        && typeof(msg.data.data.location) === 'string') {

      let statusObject = {
        status: msg.data.data.status,
        location: msg.data.data.location,
      };

      robotStatus[msg.data.data.id] = statusObject;

      if(msg.data.data.status === 'available') {
        runRobotAssignment();
      }

    } else if (msg.data.type === 'task_execution/TaskStatus'
        && typeof(msg.data.data.id) === 'string'
        && typeof(msg.data.data.status) === 'string'
        && taskStatus.hasOwnProperty(msg.data.data.id)) {

      taskStatus[msg.data.data.id]['status'] = msg.data.data.status;

    }

    return;
  } catch(e) {
    console.error("updateStatus" + e);
    res.status(400).send("Unknown error");
    return;
  }
}

function newRobot(name: string, ip: string) {
  return {
    id: {name: name, ip: ip},
    channels: [],
    lastUpdated: new Date(0),
    lastCommand: new Date(0)
  };
}

function staticRoot(req: Request, res: Response) {
  res.sendfile('', {root: 'static/live-view.html'});
}

export const roboServer = express();
roboServer.use(function(req, res, next) {
  res.header("Access-Control-Allow-Origin", "*");
  res.header("Access-Control-Allow-Headers", "Origin, X-Requested-With, Content-Type, Accept");
  next();
});
roboServer.use(bodyParser.json({ limit: '10mb' }));
roboServer.use(bodyParser.urlencoded({ extended: true, limit: '10mb' }));
roboServer.use(compression());
roboServer.get('/ServerStatus', getServerStatus);
roboServer.get('/', staticRoot);
roboServer.post('/GetData', getData);
roboServer.post('/RobotStatus', getRobotStatus);
roboServer.post('/SetData', setData);
roboServer.post('/Update', updateState);
roboServer.post('/SendRobotTo', sendRobotTo);
roboServer.post('/SendRobotCmd', sendRobotCmd);
roboServer.post('/SendPlannerCmd', sendPlannerCmd);

process.on('uncaughtException', function (err) {
  console.log(err);
}); 

// HTTP: Used for robot update
roboServer.listen(9043, () => console.log('RoboServer listening on port 9043'))

// HTTPS: Optional, Used for client-side update in production.
try {
  const privateKey = fs.readFileSync('certificate/privkey.pem', 'utf8');
  const certificate = fs.readFileSync('certificate/cert.pem', 'utf8');
  const ca = fs.readFileSync('certificate/chain.pem', 'utf8');

  const credentials = {
    key: privateKey,
    cert: certificate,
    ca: ca
  };

  https.createServer(credentials, roboServer).listen(9443);
} catch(e) {
  console.error('Unable to set up HTTPS. Only using HTTP.');
  // console.error(e);
}
