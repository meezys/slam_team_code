import { Input, Message, Point, RGBA } from "./types";

// The `./markers` utility provides a helper function to build a Marker.
import { buildRosMarker, MarkerTypes } from "./markers";

type GlobalVariables = { id: number };

export const inputs = ["/camera_cones"];
export const output = "/studio_script/cone_markers";

// Our node will output a Marker message.
type Marker = Message<"visualization_msgs/Marker">;

// If you want to output multiple markers for a single input message, use a MarkerArray.
// The marker array message has one field, `markers`, which is an array of Marker messaages.
// type MarkerArray = Message<"visualization_msgs/MarkerArray">;

let blue = { r: 0, g: 0, b: 0.9, a: 1 };
let yellow = { r: 0.9, g: 0.9, b: 0, a: 1 };
let gray = { r: 0.6, g: 0.6, b: 0.6, a: 1 };

export default function script(
  event: Input<"/camera_cones">,
  globalVars: GlobalVariables
): Marker {
  let pos_list: Point[] = [];
  let col_list: RGBA[] = [];
  event.message.cones.forEach((cone) => {
    let x = cone.radius * Math.cos(cone.angle);
    let y = cone.radius * Math.sin(cone.angle);
    let colour =
      cone.cone_type == 0 ? blue : cone.cone_type == 1 ? yellow : gray;

    pos_list.push({ x: x, y: y, z: 0 });
    col_list.push(colour);
  });

  return buildRosMarker({
    header: {
      frame_id: "camera",
      stamp: {
        sec: 0,
        nsec: 0,
      },
      seq: 0,
    },
    // Add any fields you want to set in the marker here
    // Any fields you omit will use default values
    // e.g 'type: MarkerTypes.ARROW' */
    type: MarkerTypes.CUBE_LIST,
    id: 0,
    scale: {
      x: 0.35,
      y: 0.35,
      z: 0.35,
    },
    ns: "slam",
    points: pos_list,
    colors: col_list,
  });
}
