$fn=100;

mirror([-1, 1, 0]) scale(.001) union() {
    hull() {
        translate([0, 5, 1]) cube(size=[630, 10, 2], center=true);
        for(i=[-1, 1]) {
            translate([i*(630/2-105), 221-105, 0]) cylinder(r=105, h=2);
        }
    }

    for(i=[-1, 1]) {
        translate([i*141, 15, -70]) cylinder(r=2.5, h=70);
    }
}
