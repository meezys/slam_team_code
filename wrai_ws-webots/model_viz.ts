import { Input, Message, Point, RGBA } from "./types";
import { buildRosMarker, MarkerTypes } from "./markers";

export const inputs = ["/vectornav/pose"];
export const output = "/studio_script/model";

// Our node will output a Marker message.
type Marker = Message<"visualization_msgs/Marker">;

// If you want to output multiple markers for a single input message, use a MarkerArray.
// The marker array message has one field, `markers`, which is an array of Marker messaages.
// type MarkerArray = Message<"visualization_msgs/MarkerArray">;

let blue = { r: 0, g: 0, b: 0.9, a: 1 };
let yellow = { r: 0.9, g: 0.9, b: 0, a: 1 };
let gray = { r: 0.6, g: 0.6, b: 0.6, a: 1 };

export default function script(event: Input<"/vectornav/pose">): Marker {
  return buildRosMarker({
    header: {
      frame_id: "chassis",
      stamp: event.message.header.stamp,
      seq: 0,
    },
    // Add any fields you want to set in the marker here
    // Any fields you omit will use default values
    // e.g 'type: MarkerTypes.ARROW' */
    type: MarkerTypes.MESH,
    id: 0,
    action: 0,
    scale: {
      x: 0.001,
      y: 0.001,
      z: 0.001,
    },

    ns: "model",

    pose: {
      position: { x: 0.0, y: 0.0, z: 0.0 },
      orientation: { x: 0.0, y: 0.0, z: 1, w: 0 },
    },

    mesh_resource: "file://C:/Users/Joshua/Desktop/WarwickRacing/WRai1.dae",
    color: { r: 0.0, g: 0.0, b: 0.0, a: 1.0 },
  });
}
