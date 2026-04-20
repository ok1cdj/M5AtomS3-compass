// Mount for M5Stack Atomic Proto Kit on a 25mm pipe

// --- Parameters ---
pipe_diameter = 25;         // Pipe diameter
wall_thickness = 4;         // Wall thickness

// Base dimensions for Atomic Proto Kit (24x24mm)
base_size = 24 + 2 * wall_thickness;

// M3 mounting holes for Atomic Proto (16mm spacing)
mount_hole_spacing = 16;
mount_hole_dia = 3.2;       // For M3 screws with tolerance
screw_head_dia = 5.3;       // M3 screw head diameter
screw_head_height = 3;      // M3 screw head height

// Holes for zip ties
zip_tie_hole_dia = 4;
zip_tie_hole_spacing = pipe_diameter + 12; // Distance between holes

$fn=64; // Model resolution

// Calculated Z value for the mount surface at the hole location (on the inner side of the clamp)
hole_surface_z = -pipe_diameter/2 + sqrt(pow(pipe_diameter/2, 2) - pow(mount_hole_spacing/2, 2));

// --- Model ---
difference() {
    // Main body (base + clamp)
    union() {
        // Platform for mounting the device
        translate([-base_size/2, -base_size/2, 0])
            cube([base_size, base_size, wall_thickness]);

        // Block for the pipe clamp
        translate([-zip_tie_hole_spacing/2, -base_size/2, -pipe_diameter/2])
            cube([zip_tie_hole_spacing, base_size, pipe_diameter/2 + wall_thickness]);
    }

    // Cutout for the pipe
    translate([0, 0, -pipe_diameter/2])
        rotate([90, 0, 0])
            cylinder(d=pipe_diameter, h=base_size+1, center=true);

    // Mounting holes for Atomic Proto Kit (M3) - through the entire part
    translate([mount_hole_spacing/2, 0, -pipe_diameter/2 - 1])
        cylinder(d=mount_hole_dia, h = wall_thickness + pipe_diameter/2 + 2);
    translate([-mount_hole_spacing/2, 0, -pipe_diameter/2 - 1])
        cylinder(d=mount_hole_dia, h = wall_thickness + pipe_diameter/2 + 2);

    // Countersink for screw heads on the bottom side (by the pipe)
    translate([mount_hole_spacing/2, 0, -pipe_diameter/2 - 1])
        cylinder(d=screw_head_dia, h = hole_surface_z + screw_head_height + pipe_diameter/2 + 1);
    translate([-mount_hole_spacing/2, 0, -pipe_diameter/2 - 1])
        cylinder(d=screw_head_dia, h = hole_surface_z + screw_head_height + pipe_diameter/2 + 1);

    // Holes for zip ties
    translate([zip_tie_hole_spacing/2 - wall_thickness - zip_tie_hole_dia/2, 0, -pipe_diameter])
        cylinder(d=zip_tie_hole_dia, h=pipe_diameter*2);
    translate([-zip_tie_hole_spacing/2 + wall_thickness + zip_tie_hole_dia/2, 0, -pipe_diameter])
        cylinder(d=zip_tie_hole_dia, h=pipe_diameter*2);
}
