//use <../inc/heat_block.scad>


// general object resolution
$fn = 50;

// the extruder design heavily depends on the shape of the syringe
// and therefore uses the configuration values from inc/syringe to
// generate the parts. Make sure those values fit your syringe precisely!
//include <../inc/syringe.scad>

profile_height = 13;
profile_offset = 0.2;

shadow_diameter = 135.0;
shadow_disk_height = 12.0;

scale(1/1000) rotate([-90,0,0]) rotate([0,-90,0]) shadow_adapter();

//trixi_profile();

//#translate([0, 0, 17.4]) rotate([0, 180, 0]) import(file = "trixi_original_adapter.stl");

module shadow_adapter() {
	union() {
		difference() {
			//main disk
			union() {
				cylinder(r=shadow_diameter/2, h=shadow_disk_height, center=false, $fn=100);
				translate([0, 0, shadow_disk_height]) cylinder(r=50, h=profile_height, center=false, $fn=100);
			}
			for(i=[0:7]) {
				rotate([0, 0, i*360/8+45/2]) translate([50.0, 0, -7.0]) shadow_screw(head=25);
			}

			//trixi interface disk
			translate([0, 0, shadow_disk_height]) {
				trixi_profile();
			}

			//wire opening
			translate([0, 5, 0]) cylinder(r=30, h=60, center=true);
			translate([0, 17, 0]) {
				hull() {
					for(i=[0:4]) {
						rotate([0, 0, 90*i+45]) translate([20, 0, 0]) cylinder(r=5, h=100, center=true);
					}
				}
			}
		}
	}
}

module shadow_screw(head = 10){
	union(){
		cylinder(r=3.4, h=12, center=false);
		translate([0, 0, 11.99]) cylinder(r=5.4, h=head, center=false);
	}
}

module trixi_profile() {
	union() {
		linear_extrude(height = profile_height+0.1, center = false, convexity = 10) {
			offset(r=0.5+profile_offset) {
				import (file = "adapter_surface_original.dxf");
			}
		}

		//screws
		for(i = [-1, 1]) {
			translate([0, 0, profile_height-7.6]) {
				translate([i*40, 11, 0]) rotate([0, 90, 0]) {
					cylinder(r=2.7, h=30, center=true);
					translate([0, 0, i*11]) cylinder(r=5, h=10, center=true);
				}
				translate([i*30, -33, 0]) rotate([0, 90, 0]) {
					cylinder(r=2.7, h=30, center=true);
					translate([0, 0, i*7]) cylinder(r=5, h=20, center=true);
				}
			}
		}
	}
}
