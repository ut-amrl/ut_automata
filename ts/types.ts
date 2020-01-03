export type RobotID = {
  // Human-readable name of the robot.
  name: string,
  // IP address on VPN for the robot.
  ip: string
};

// JSON representation of robot channel data.
export type JsonString = string;

// Specialized data format for image channel data.
export type ImageChannelData = Buffer;

// Allowed types for robot channels.
export type RobotChannelType = ImageChannelData | JsonString;

// Description of a channel.
export type RobotChannelDescription = {
  // Name, analogous to a ROS topic name.
  name: string,
  // Type, analogous to ROS msg name.
  type: string,
  // Maximum rate at which messages will be accepted.
  maxRate: number
}

// A robot channel, equivalent to a ROS topic.
export type RobotChannel = {
  // Name, analogous to a ROS topic name.
  name: string,
  // Type, analogous to ROS msg name.
  type: string,
  // Payload data. Could be a specialized format (e.g. ImageData) for 
  // efficiency, or a JsonString for all others.
  data: RobotChannelType
}

// Current robot state.
export type RobotState = {
  id: RobotID,
  channels: RobotChannel[],
  lastUpdated: Date,
  lastCommand: Date
};

// Summary of robot state.
export type RobotStateSummary = {
  id: RobotID,
  channels: string[],
  lastUpdated: string,
  lastCommand: string
};

// Update from robot for a specific topic.
export type RobotUpdate = {
  // Name of the robot, as in RobotID.
  name: string,
  // Robot channel update.
  data: RobotChannel
}