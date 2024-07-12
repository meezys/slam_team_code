import { Input, Message, Time } from "./types";
import { buildRosMarker, MarkerTypes } from "./markers";

export const inputs = ["/cones"];
export const output = "/studio_script/cone_models";

// Our node will output a Marker message.
type Marker = Message<"visualization_msgs/Marker">;

// If you want to output multiple markers for a single input message, use a MarkerArray.
// The marker array message has one field, `markers`, which is an array of Marker messaages.
type MarkerArray = Message<"visualization_msgs/MarkerArray">;

type Cone = {
  x: number;
  y: number;
  z: number;
};

function buildCone(cone: Cone, stamp: Time, mesh_resource: string, id: number) {
  return buildRosMarker({
    header: {
      frame_id: "track",
      stamp: stamp,
      seq: 0,
    },
    // Add any fields you want to set in the marker here
    // Any fields you omit will use default values
    // e.g 'type: MarkerTypes.ARROW' */
    type: MarkerTypes.MESH,
    id: id,
    action: 0,
    scale: {
      x: 1.0,
      y: 1.0,
      z: 1.0,
    },
    ns: "cones",
    pose: {
      position: {
        x: cone.x,
        y: cone.y,
        z: cone.z,
      },
      orientation: { x: 0.0, y: 0.0, z: 0, w: 1 },
    },
    mesh_resource: mesh_resource,
    mesh_use_embedded_materials: true,
  });
}

export default function script(event: Input<"/cones">): MarkerArray {
  let markers: Array<Marker> = [];
  let id = 0;

  event.message.yellow_cones.forEach((cone) => {
    let marker = buildCone(
      cone,
      event.message.header.stamp,
      "file://C:/Users/Joshua/Desktop/WarwickRacing/cone_yellow.dae",
      id
    );
    id += 1;
    markers.push(marker);
  });

  event.message.blue_cones.forEach((cone) => {
    let marker = buildCone(
      cone,
      event.message.header.stamp,
      "file://C:/Users/Joshua/Desktop/WarwickRacing/cone_blue.dae",
      id
    );
    id += 1;
    markers.push(marker);
  });

  return {
    markers: markers,
  };
}
