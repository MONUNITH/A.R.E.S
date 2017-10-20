//This number changes the number of sides the model uses. Larger numbers make the model smoother but will take longer to render.
$fn = 64;


// Motor-Rad
motor_ring_z = 4; // Rad-Höhe (mm)
motor_ring_r1 = 18/2; // Rad Außenradius (mm)
motor_ring_r2 = 2.0/2; // Rad Innenradius (mm)
motor_ring_thick = 2.5; // Ring Höhe (mm)


// Ring
module ring( r1, r2, z ){
  difference(){
      cylinder(h = z, r=r1, center = false);
      translate([0,0,-1]) cylinder(h = z+2, r=r2, center = false);
  }    
}    

// Rundring
module ring_disc( r1, r2, z, thickness ){
  difference(){      
    ring(r1, r2, z);
    rotate_extrude(convexity = 10)    
	  translate([r1,  2,0])
	  circle(r = thickness/2);
  }      
}    


// Motor-Rad
translate([0,0,0]) ring_disc(motor_ring_r1, motor_ring_r2, motor_ring_z, motor_ring_thick);  

