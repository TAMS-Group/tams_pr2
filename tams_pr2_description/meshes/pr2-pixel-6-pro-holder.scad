/** pr2-pixel-6-pro-holder.scad
 *
 * Simple adapter to mount a Google Pixel 6 Pro phone 
 * to the head of the WillowGarage PR2 robot.
 *
 * 28.06.2022 - v3, allow right holder to actually rotate...
 * 26.06.2022 - v2, rounded holder, stabilizer, "langloch"
 * 21.06.2022 - v1, basic cube-shaped version
 * -
 * (c) 2022 fnh, hendrich@informatik.uni-hamburg.de
 */
 

dd = 25.4; // distance between screw holes/threads on PR2 head

show_head_mesh = 1;
show_head_model = 1;
show_pixel_6_pro = false;
show_pixel_6_pro_with_cover = 1;

left_holder  = 1;         // PR2 left-eye 
right_holder = 1;        // PR2 right-eye
make_right_mezzanine_plate = 0;
make_mezzanine_plates = false;


if (show_pixel_6_pro_with_cover)
translate( [0, xxx, zzz] ) { 
pixel_6_pro_with_silicon_cover( pixel_cutout=true );
translate( [0,0,0] ) pixel_6_pro();
}


// the PR2 head model plus the WillowGarage mesh aligned
// to be at the same position (centered around origin)
//
if (show_head_mesh) {
color( [0.7,0.7,0.7] ) 
translate( [0, -38, -115] ) rotate( [0,0,-90] ) scale( 100,100,100 )
  import( "head_tilt.stl" );
} 
if (show_head_model) {
  pr2_head();
}


// the model of the Pixel 6 Pro, plus the Thingiverse mesh
// aligned to be at the same position (xxx, yyy, zzz).
//
xxx = -116.3;
zzz =   55;

if (show_pixel_6_pro) {
translate( [0, xxx, zzz] ) pixel_6_pro();
color( [0.7,0.7,0.7] ) 
translate( [0, xxx, zzz] ) rotate( [0,90,90] ) scale( 1,1,1 )
  import( "Pixel_6_Pro_Body.stl" );
}


// the model of the Pixel 6 Pro with silicon-cover
// aligned to be at (0, -120, zzz).
// 
if (false)
translate( [0, xxx, zzz] ) { 
pixel_6_pro_with_silicon_cover( pixel_cutout=true );
translate( [0,1,0] ) pixel_6_pro();
}


// "distance plates" with screw holes to be mounted
// below the Azure-Kinect holder
//
if (make_mezzanine_plates) {
translate( [+3*dd, -63, 80/2+5/2] ) 
  pr2_head_mezzanine_plate( size=[25.4+2*10, 25.4+20, 5] );

translate( [-3*dd, -63, 80/2+5/2] ) 
  pr2_head_mezzanine_plate( size=[25.4+2*10, 25.4+20, 5] );
}


if (make_right_mezzanine_plate) {
  h_head = 80.0;
  difference() {
    color( "black" )
    translate( [-2.5*dd, -63, 80/2+5/2] ) 
      pr2_head_mezzanine_plate( size=[25.4-2, 25.4+20, 5] );
    // subtract holes for head mount plate screws
    translate( [0,0,h_head/2-5.0/2+0.1] ) pr2_head_mount_plate( holes=true, M6_diameter=8 );
  }
}




if (left_holder) 
difference() {
pr2_pixel_6_pro_holder(  3*dd, sign=+1, usb_cutout=[20,12,15], langloch=false );
translate( [0, xxx, zzz] ) { 
  pixel_6_pro_with_silicon_cover( pixel_cutout=false );
  // translate( [0,1,0] ) pixel_6_pro();
}}


if (right_holder) {
    // cylinder( d=50, h=130, center=true );
difference() {
  pr2_pixel_6_pro_holder(  3*dd, sign=-1, usb_cutout=[1,1,1], langloch=true ); // -3.0

  if (true) 
  translate( [0, xxx, zzz] ) { 
    pixel_6_pro_with_silicon_cover( pixel_cutout=false );
    // translate( [0,1,0] ) pixel_6_pro();
  }
}
}



// ################################
// ################################


module pr2_pixel_6_pro_holder( dy=2.5*dd, sign=1, usb_cutout=[0,0,0], langloch=false )
{
  dd = 25.4;
  h_head  = 80; // zzz + 54;
  d_lever = 21;  // length of holder bar in front of the head
  d_mount = 18;  // y-thickness (front-back) of holder part
  h_mount = 84;
  _rr  =  6;     // corner radius of holder block
    
  difference() {
    union() {  
      // part on head-mounting plate with holes
      if (langloch == false) {
        translate( [sign*dy,-2.5*dd, h_head/2+5.0/2] )
          cube( [16,dd+2*10,5], center=true );
        translate( [sign*dy - sign*4*2.54,-2.5*dd, h_head/2+5.0/2] )
          cube( [25.4,dd+2*10,5], center=true );
      }
    
      // connecting lever
      translate( [sign*dy,-3.0*dd-10-d_lever/2, h_head/2+5.0/2] )
        cube( [16,d_lever,5], center=true );
        
      // extra connecting part to componensate rounding-corners 
      translate( [sign*dy,-3.0*dd-10-d_lever- _rr/2, h_head/2+5.0/2] )
        cube( [16,_rr,5], center=true );

      // "vertical stabilizer" part
      hull() {
        _ddy = [d_lever/2, -d_lever/2-_rr, -d_lever/2-_rr];
        _ddz = [ 0, 0, 10];
        for( i=[0:2] )
           translate( [sign*dy, -3.0*dd-10-d_lever/2+_ddy[i], h_head/2+5.0/2+_ddz[i]] )
             cube( [16,1,1], center=true );
      }  

      // phone holding part
      if (0) 
      color( "gold", 0.1 )
      translate( [sign*dy,-3.0*dd-10-d_lever-d_mount/2, zzz + 0.0] )
        cube( [16,d_mount,h_mount], center=true );
        
      // use spheres to make the block look nicer
      translate( [sign*dy,-3.0*dd-10-d_lever-d_mount/2, zzz + 0.0] ) {
        _ddx = 18/2 - _rr;
        _ddy = d_mount/2 - _rr;
        _ddz = h_mount/2 - _rr;
        hull() {
          for( _dddx = [- _ddx, + _ddx] )
            for( _dddy = [- _ddy, + _ddy] )
              for( _dddz = [- _ddz, + _ddz] ) 
                translate( [_dddx, _dddy, _dddz] )
                  sphere( r = _rr, $fn=20 );
          } 
      }

      // connecting lever
      if (langloch) {
        // part on head-mounting plate with holes
          
//        color( "blue" )
//        translate( [sign*dy,-2.5*dd, h_head/2+5.0/2] )
//          cube( [16,dd+2*10,14], center=true );

        color( "red" )   
        hull() {
          translate( [sign*dy+sign*0.5*dd, -2.0*dd, h_head/2+7.0/2] )
             cylinder( d=25.4, h=7, center=true );
          translate( [sign*dy+sign*0.5*dd, -3.0*dd, h_head/2+7.0/2] )
             cube( [25.4, 20.0, 7], center=true );
        }
        
        // connecting lever
        hull() {
          translate( [sign*dy+sign*0.5*dd,-3.0*dd-10, h_head/2+7.0/2] )
            cube( [25.4,1,7], center=true );
            
          translate( [sign*dy,-3.0*dd-10-d_lever/2, h_head/2+7.0/2] )
            cube( [16,d_lever,7], center=true );
        }
        
      // extra connecting part to componensate rounding-corners 
      translate( [sign*dy,-3.0*dd-10-d_lever- _rr/2, h_head/2+5.0/2] )
        cube( [16,_rr,5], center=true );
        
//        color( "green" )   
//        translate( [sign*dy+sign*dd - sign*4*2.54,-2.5*dd, h_head/2+5.0/2] )
//           cube( [25.4,dd+2*10,5], center=true );
      }
        
    } // union
    
    // extra cutout for USB connector (left eye)
    translate( [sign*dy,-3.0*dd-10-d_lever-d_mount/2, zzz + 0.0] )
      cube( [usb_cutout[0], usb_cutout[1], usb_cutout[2]], center=true );
    
    // subtract holes for head mount plate screws
    translate( [0,0,h_head/2-5.0/2+0.1] ) pr2_head_mount_plate( holes=true, M6_diameter=8 );


    // "langloch" to rotate right-eye holder outwards
    if (langloch) 
    translate( [sign*3.5*dd, -2*dd, h_head/2-0.5] )
      for( theta=[-40:0] ) {
        rotate( [0,0,180+sign*theta] )
          translate( [0,dd,0] )      
           cylinder( d=8, h=8, center=false, $fn=30 );
      }
  } // difference
}
 

// ################################
// ################################



module pr2_head( show_neopixel_rings=false ) {
  w_head = 295;
  l_head = 6*dd+2*10+2*20;
  h_head = 80;

  difference() {
    color( [1,1,1,0.3] )    
    cube( [w_head, l_head, h_head], center=true );
    translate( [0,0,h_head/2-5.0/2+0.1] ) pr2_head_mount_plate( holes=false );
  }   
  
  // translate( [0,0,h_head/2-5.0/2+0.1] ) pr2_head_mount_plate();
  
  // neopixel rings around the "eyes" (prosilica and IR projector)
  if (show_neopixel_rings) {
    translate( [+109.5,-l_head/2-5,4] ) 
      neopixel_ring_16();  
    translate( [-109.5,-l_head/2-5,4] ) 
      neopixel_ring_16();  
  }
}


module pr2_head_mount_plate( holes=true, M6_diameter=6.2, M6_length=15, M6_head=0 ) {
  dd = 25.4;
  hh =  5.0;
  difference() {
    color( [0.8,0.8,0.8] ) 
      cube( [9*dd + 2*10.0, 6*dd+2*10.0, hh], center=true);
      
    if (holes == true) {
      dxx = [0.5*dd, 1.5*dd, 2.5*dd, 3.5*dd, 4.5*dd];
      dyy = [-3*dd, -2*dd, -1*dd, 0, 1*dd, 2*dd, 3*dd];
      for( dx=dxx ) {
        for( dy = dyy ) {
          // echo( dx );   
          translate( [ dx,dy,0] ) 
            cylinder( d=M6_diameter, h=hh+1, $fn=50, center=true );
           translate( [-dx,dy,0] ) 
             cylinder( d=M6_diameter, h=hh+1, $fn=50, center=true );
        }
      }
    } // if 
  } // difference
  
  // M6 screws for later difference operation
  if (M6_length > 0) {
      dxx = [0.5*dd, 1.5*dd, 2.5*dd, 3.5*dd, 4.5*dd];
      dyy = [-3*dd, -2*dd, -1*dd, 0, 1*dd, 2*dd, 3*dd];
      for( dx=dxx ) {
        for( dy = dyy ) {
          // echo( dx );   
          translate( [ -dx,dy,0] ) 
            cylinder( d=M6_diameter, h=M6_length, $fn=50, center=false );
           translate( [-dx,dy,M6_length] ) 
             cylinder( d=11.0, h=M6_head, $fn=50, center=false );
          translate( [ dx,dy,0] ) 
            cylinder( d=M6_diameter, h=M6_length, $fn=50, center=false );
           translate( [ dx,dy,M6_length] ) 
             cylinder( d=11.0, h=M6_head, $fn=50, center=false );
        }
      }
  } // if 
} // module
 


module wheel_color( wheel_pos = 0 ) {
  wheel_pos = 255 - wheel_pos;
  if (wheel_pos < 85) {
    rgb = [ (255 - wheel_pos*3)/256.0, 0.0, (wheel_pos*3)/256.0 ];
  }
  else if (wheel_pos < 170) {
    wheel_pos = wheel_pos - 85;
    rgb = [ 0.0, (wheel_pos*3)/256.0, (255 - wheel_pos*3)/256.0 ];
  }
  else {
    wheel_pos = wheel_pos - 170;
    rgb = [ (wheel_pos*3)/256.0, (255 - wheel_pos*3)/256.0, 0.0 ];
  }
  // color( rgb ) children();
  rgb = [1, wheel_pos/160.0, 1]; 
  color( rgb ) children();
}



//function up_down_zero( i, imax ) =
//  (i % imax) / imax;

function up_down_zero( i, imax ) =
  (i < (imax/3)) 
  ? (3.0 * i / imax)
  : (i < (2.0*imax/3.0)) 
    ? ((imax-3*i) / imax)
    : 0.8;


module neopixel_ring_16() {
  led_diameter   =  7.0;
  outer_diameter = 44.5;
  inner_diameter = 31.7;
  led_radius = 20;
    
  phi0 = 0; dphi = 360.0/16.0;
  rotate( [90,90,0] ) 
  for( i=[0:15] ) {
    echo( i );   
    rotate( [0,0,i*dphi] )
      translate( [led_radius,0,0] ) 
      color( [up_down_zero( i%16, 16), 
              up_down_zero( (i+6)%16, 16),
              up_down_zero( (i+12)%16, 16 )] )
        cube( [led_diameter,led_diametertrue,4.0], center=true );
  }
}

 





module pr2_head_mezzanine_plate( size=[15, 25.4+20, 5] )
{
  cube( size, center=true );
}

 

module pixel_6_pro( 
  show_straight_usb_connector = true, 
  show_angled_usb_connector = false, 
)
{
  corner_radius = 4.0;
  xx = 163.9 - 2*corner_radius;
  yy = 8.9 - 2*corner_radius; 
  zz = 75.8 - 2*corner_radius;

  // main phone body
  color( [.9, .9, 0.95] ) 
  minkowski() {
    cube( size=[xx,yy,zz], center=true );
    sphere( r=corner_radius, $fn=50 );
  }

  // a thin black display
  color( [0,0.2,0] ) 
  translate( [0,8.9/2,0] ) 
    cube( size=[145,0.5,65], center=true );

  dx = -163.9/2 + 17.7 + 25.2/2 -2;
  color( [0.1, 0.1, 0.1] ) 
  translate( [dx, -2.5, 0] )
    minkowski() {
      rotate( [0,90,0] ) 
        cylinder( r=4, h=0.01, $fn=20, center=true );
      cube( size=[26.2, 10-8, zz], center=true );
    }
  // TODO: model cameras as slight dimples / holes
    
  if (show_straight_usb_connector) { // 6x11x18 cable d=5
    xu = 18;
    translate( [163.9/2+xu/2-1, 0, 0] )
      minkowski() {
        ds = 3;
        sphere( d=ds, $fn=30 );
        cube( [xu-ds,7+1-ds,11-ds], center=true ); 
      }
    translate( [163.9/2+xu-1, 0, 0] )
      rotate( [0,90,0] )
        cylinder( d=5, h=20, center=false, $fn=30 );
  }
  if (show_angled_usb_connector) { // 7x13x18x23 + straight connector
    xu = 18;
    color( [1,1,1] )
    translate( [163.9/2+xu/2-0.1, 0, 0] ) {
      cube( [xu,7+1,13], center=true ); 
    }      
    color( [1,1,1] )
    translate( [163.9/2+xu-14/2, 0, 23/2-13/2] ) {
      cube( [14,8.1,23], center=true ); 
    }    
    
    // the straight connector comes for free :-)
    //
    translate( [163.9/2+xu-14/2, 0, 23-13/2+18/2] ) {
      cube( [13,7+1,18], center=true ); 
    }      
    translate( [163.9/2+xu-14/2, 0, 23-13/2+18] )
      rotate( [0,0,0] )
        cylinder( d=5, h=20, center=false, $fn=30 );
  }
}

module pixel_6_pro_with_silicon_cover( pixel_cutout=true )
{
  corner_radius = 4.0;
  xx = 168.0 - 2*corner_radius;
  yy =  12.1 - 2*corner_radius;
  zz =  80.2 - 2*corner_radius;

  color( [.9, .9, 0.7,  0.8] ) 
  difference() {
    minkowski() {
      cube( size=[xx,yy,zz], center=true );
      sphere( r=corner_radius, $fn=50 );
    }
    if (pixel_cutout) 
      translate( [0, 2, 0] ) 
        pixel_6_pro();
  }
  
  
}    