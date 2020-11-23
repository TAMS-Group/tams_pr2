module cyl(y,z){
  translate([5,y,z]){
    rotate([0,90,0]){
      cylinder(h=16,r=2.9);
    }
  }
}


module four_cyl(h,w,z){
  cyl(w,z+0);
  cyl(-w,z+0);
  cyl(w,z+h);
  cyl(-w,z+h);
}

four_cyl(94+5.8,37,3);

translate([20,-55,-2]){
  cube([3,110,110]);
}

/*
difference(){
  translate([-67.29*0.707,0,0]) {
    cylinder(h=100, r=70.29);
  }
  translate([-67.29*0.707,0,-5]) {
    cylinder(h=110, r=67.29);
  }
  translate([-150,-75,-1]) {
    cube([150,150,150]);
  }
}
*/