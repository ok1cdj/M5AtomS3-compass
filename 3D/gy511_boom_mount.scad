// Holder for the GY-511 (LSM303D) compass module on a 25 mm antenna boom, fixed with zip ties.
// Datum: boom axis along Y through the origin, holder on top of the boom (+Z).
// The module Y arrow points along +Y (antenna front); the arrow on the lid shows it.
// Print the holder upside down (top face on the bed) and the lid flat, arrow up: no supports.

part = "assembly"; // "holder", "lid" or "assembly"

// --- Boom ---
pipe_d = 25;            // boom diameter
pipe_clearance = 0.2;   // per side
saddle_depth = 5;       // how far the saddle wraps down from the top of the boom

// --- GY-511 module (measured) ---
pcb_x = 21;             // long edge, across the boom
pcb_y = 15;             // short edge = module Y axis, along the boom
pcb_t = 1.6;            // PCB thickness
module_h = 4;           // total height incl. components and solder joints
pcb_clearance = 0.2;    // per side, glued in place
glue_gap = 0.3;         // between the module top and the lid

// --- Flat cable, leaves the long edge along the boom ---
cable_w = 6;
cable_t = 1.5;
cable_clearance = 0.5;  // per side
cable_side = 1;         // -1: cable leaves towards -Y (away from the arrow), +1: towards +Y

// --- Body ---
floor_t = 2.0;          // minimum between the boom and the pocket floor
wall = 2.4;             // pocket side walls (6 perimeters at 0.4 mm)

// --- Lid, sunk into a recess and glued ---
lid_t = 1.5;
lid_lip = 1.0;          // how far the lid overlaps the wall top
lid_clearance = 0.15;   // per side
arrow_depth = 0.6;      // engraved direction arrow

// --- Zip ties, run through closed tunnels above the boom: tightening pulls the holder down ---
strap_w = 5;            // up to 4.8 mm zip ties
strap_t = 1.5;
strap_clearance = 0.4;  // per side
strap_margin = 2.0;     // material between a tunnel and the holder end
tunnel_gap = 2.0;       // material between the boom and a tunnel, pressed down by the strap
groove_bridge = 1.2;    // material between the cable groove and the tunnel below it

eps = 0.01;
$fn = 96;

// --- Derived ---
pipe_r = pipe_d / 2 + pipe_clearance;
pocket_x = pcb_x + 2 * pcb_clearance;
pocket_y = pcb_y + 2 * pcb_clearance;
pocket_depth = module_h + glue_gap;
tunnel_h = strap_t + 2 * strap_clearance;
tunnel_bottom_z = pipe_r + tunnel_gap;
tunnel_top_z = tunnel_bottom_z + tunnel_h;
// The cable groove crosses over a tunnel, so the pocket is raised above it when needed
floor_z = max(pipe_r + floor_t, tunnel_top_z + groove_bridge - pcb_t);
recess_z = floor_z + pocket_depth;
top_z = recess_z + lid_t;
bottom_z = pipe_d / 2 - saddle_depth;

recess_x = pocket_x + 2 * lid_lip;
recess_y = pocket_y + 2 * lid_lip;
lid_x = recess_x - 2 * lid_clearance;
lid_y = recess_y - 2 * lid_clearance;

tunnel_y = strap_w + 2 * strap_clearance;
tunnel_center_y = pocket_y / 2 + wall + tunnel_y / 2;

body_x = pocket_x + 2 * wall;
body_y = 2 * (tunnel_center_y + tunnel_y / 2 + strap_margin);

cable_slot_w = cable_w + 2 * cable_clearance;
cable_floor_z = floor_z + pcb_t; // cable is soldered on top of the PCB

echo(body = [body_x, body_y, top_z - bottom_z], pocket = [pocket_x, pocket_y, pocket_depth],
     lid = [lid_x, lid_y, lid_t], top_z = top_z);

assert(wall - lid_lip >= 1.2, "recess wall too thin");
assert(cable_floor_z - tunnel_top_z >= groove_bridge - eps, "cable groove breaks into a strap tunnel");
assert(top_z - tunnel_top_z >= 2, "too little material above a strap tunnel");
assert(body_x / 2 > sqrt(pipe_r * pipe_r - bottom_z * bottom_z), "saddle wider than the body");
assert(cable_slot_w < pocket_x, "cable slot wider than the pocket");

module holder() {
  difference() {
    translate([-body_x / 2, -body_y / 2, bottom_z])
      cube([body_x, body_y, top_z - bottom_z]);

    // Saddle
    rotate([90, 0, 0])
      cylinder(r = pipe_r, h = body_y + 2 * eps, center = true);

    // Module pocket
    translate([-pocket_x / 2, -pocket_y / 2, floor_z])
      cube([pocket_x, pocket_y, pocket_depth + eps]);

    // Lid recess
    translate([-recess_x / 2, -recess_y / 2, recess_z])
      cube([recess_x, recess_y, lid_t + eps]);

    // Closed zip tie tunnels across the body, above the boom
    for (s = [-1, 1])
      translate([-body_x / 2 - eps, s * tunnel_center_y - tunnel_y / 2, tunnel_bottom_z])
        cube([body_x + 2 * eps, tunnel_y, tunnel_h]);

    // Cable groove from the pocket to the holder end, along the boom
    translate([-cable_slot_w / 2,
               cable_side > 0 ? pocket_y / 2 - eps : -body_y / 2 - eps,
               cable_floor_z])
      cube([cable_slot_w, body_y / 2 - pocket_y / 2 + 2 * eps, top_z - cable_floor_z + eps]);
  }
}

module arrow_2d() {
  // Points to +Y; shaft and head scaled to the lid
  head_w = lid_x * 0.5;
  head_l = lid_y * 0.35;
  shaft_w = lid_x * 0.18;
  shaft_l = lid_y * 0.4;
  translate([0, -(head_l + shaft_l) / 2]) {
    translate([-shaft_w / 2, 0]) square([shaft_w, shaft_l + eps]);
    translate([0, shaft_l]) polygon([[-head_w / 2, 0], [head_w / 2, 0], [0, head_l]]);
  }
}

module lid() {
  difference() {
    translate([-lid_x / 2, -lid_y / 2, 0]) cube([lid_x, lid_y, lid_t]);
    translate([0, 0, lid_t - arrow_depth])
      linear_extrude(arrow_depth + eps) arrow_2d();
  }
}

if (part == "holder") {
  holder();
} else if (part == "lid") {
  lid();
} else {
  holder();
  translate([0, 0, recess_z]) lid();
  // Ghosts: boom and module
  %rotate([90, 0, 0]) cylinder(d = pipe_d, h = body_y * 2, center = true);
  %translate([-pcb_x / 2, -pcb_y / 2, floor_z]) cube([pcb_x, pcb_y, module_h]);
}
