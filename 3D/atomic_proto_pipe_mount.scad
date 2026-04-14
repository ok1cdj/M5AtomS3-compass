// Držák pro M5Stack Atomic Proto Kit na trubku 25mm

// --- Parametry ---
pipe_diameter = 25;         // Průměr trubky
wall_thickness = 3;         // Tloušťka stěn

// Rozměry základny pro Atomic Proto Kit (24x24mm)
base_size = 24 + 2 * wall_thickness;

// Montážní otvory M3 pro Atomic Proto (rozteč 16mm)
mount_hole_spacing = 16;
mount_hole_dia = 3.2;       // Pro M3 šrouby s tolerancí
screw_head_dia = 5.3;       // Průměr hlavy šroubu M3
screw_head_height = 3;      // Výška hlavy šroubu M3

// Otvory pro stahovací pásky
zip_tie_hole_dia = 4;
zip_tie_hole_spacing = pipe_diameter + 12; // Vzdálenost otvorů od sebe

$fn=64; // Rozlišení modelu

// Vypočtená hodnota Z pro povrch držáku v místě díry (na vnitřní straně objímky)
hole_surface_z = -pipe_diameter/2 + sqrt(pow(pipe_diameter/2, 2) - pow(mount_hole_spacing/2, 2));

// --- Model ---
difference() {
    // Hlavní tělo (základna + objímka)
    union() {
        // Plošina pro přišroubování zařízení
        translate([-base_size/2, -base_size/2, 0])
            cube([base_size, base_size, wall_thickness]);

        // Blok pro objímku na trubku
        translate([-zip_tie_hole_spacing/2, -base_size/2, -pipe_diameter/2])
            cube([zip_tie_hole_spacing, base_size, pipe_diameter/2 + wall_thickness]);
    }

    // Výřez pro trubku
    translate([0, 0, -pipe_diameter/2])
        rotate([90, 0, 0])
            cylinder(d=pipe_diameter, h=base_size+1, center=true);

    // Montážní otvory pro Atomic Proto Kit (M3) - skrz celý díl
    translate([mount_hole_spacing/2, 0, -pipe_diameter/2 - 1])
        cylinder(d=mount_hole_dia, h = wall_thickness + pipe_diameter/2 + 2);
    translate([-mount_hole_spacing/2, 0, -pipe_diameter/2 - 1])
        cylinder(d=mount_hole_dia, h = wall_thickness + pipe_diameter/2 + 2);

    // Zapuštění pro hlavy šroubů na spodní straně (u trubky)
    translate([mount_hole_spacing/2, 0, hole_surface_z])
        cylinder(d=screw_head_dia, h=screw_head_height);
    translate([-mount_hole_spacing/2, 0, hole_surface_z])
        cylinder(d=screw_head_dia, h=screw_head_height);

    // Otvory pro protažení stahovacích pásků
    translate([zip_tie_hole_spacing/2 - wall_thickness - zip_tie_hole_dia/2, 0, -pipe_diameter])
        cylinder(d=zip_tie_hole_dia, h=pipe_diameter*2);
    translate([-zip_tie_hole_spacing/2 + wall_thickness + zip_tie_hole_dia/2, 0, -pipe_diameter])
        cylinder(d=zip_tie_hole_dia, h=pipe_diameter*2);
}
