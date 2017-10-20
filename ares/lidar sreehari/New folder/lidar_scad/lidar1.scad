// LidarLiteV2 chassis (modular, small size)

//$fa=0.5; // default minimum facet angle is now 0.5
//$fs=0.5; // default minimum facet size is now 0.5 mm

//This number changes the number of sides the model uses. Larger numbers make the model smoother but will take longer to render.
$fn = 64;

// base
drawBase = true;
drawBasePlate = false;
drawBearing = false;
drawBearingBase = false;
drawBearingRing = false;
drawEncoder = false;

// disc and cover
drawDisc = true;
drawDiscTop = true;
drawLidar = false;
drawDiscCover = false;
drawDiscCoverTop = false;
drawSlipRing = false;

// motor ring
drawMotorRing = false;

disc_r = 70/2; // Außen-Radius (mm)

// Kugellager
bear_r1 = 47/2; // Außen-Radius (mm)
bear_r2 = 35/2; // Innen-Radius (mm)
bear_z = 7; // Höhe (mm)
bear_z1 = -4; // z-Position

// Encoder
enc_r = 59/2; // Radius (mm)
enc_z = 3; // Höhe (mm)

// Schleifkontakt
slip_r1 = 24/2; // Außen-Radius (mm)
slip_r2 = 18/2; // Befestigungs-Löcher-Radius (mm)
slip_r3 = 5/2;  // Unterseite-Radius (mm)
slip_r4 = 12.5/2;  // Oberseite-Radius (mm)
slip_z = 13.5; // Höhe (mm)

// Grundplatte
ground_x=70; // Länge (mm)
ground_y=97; // Breite  (mm)
ground_z=3;  // Höhe (mm)

// Gummi-O-Ring
ring_r = 70 / 2; // Gummi-Innenradius (mm)
ring_thick = 2.5; // Gummi-Dicke (mm)

// Lidar
lidar_cyl_x = 1.89*25.4;  // Lidar Cyl Breite (mm)
lidar_cyl_y = 1*25.4;  //Lidar Cyl Länge (mm)
lidar_cyl_r = 0.79*25.4/2;  // Lidar Cyl Radius (mm)
lidar_box_x = 1.9*25.4; // Lidar Box Breite (mm)
lidar_box_y = 0.47*25.4; // Lidar Box Länge (mm)
lidar_box_z = 0.79*25.4; // Lidar Box Höhe (mm)
lidar_back_x = 1.90*25.4; // Lidar Back Breite (mm)
lidar_back_y = 0.10*25.4; // Lidar Back Länge (mm)
lidar_back_z = 1.40*25.4; // Lidar Back Höhe (mm)

// Cover
cover_r = enc_r + 4;  // Cover-Radius (mm)
cover_h = 35; // Cover-Höhe (mm)

// Motorhalterung
motor_r = 25/2;   // Motor-Radius (mm)
motor_z = 2;  // Motor-Höhe (mm)

// Motor-Rad
motor_ring_r1 = 8; // Rad Außenradius (mm)
motor_ring_r2 = 2/2; // Rad Innenradius (mm)
motor_ring_thick = 2.5; // Rad Höhe (mm)


// Rechteck
module rectangle(w, h, thickness, z){
  difference(){
      cube([w, h, z], center=false);
      translate([thickness/2,thickness/2,-1]) cube([w-thickness, h-thickness, z+3], center=false);
  } 
}    

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


// Kreissegment
module arc( height, depth, radius, degrees ) {
    // This dies a horible death if it's not rendered here
    // -- sucks up all memory and spins out of control
    render() {
        difference() {
            // Outer ring
            rotate_extrude($fn = 100)
                translate([radius - height, 0, 0])
                    square([height,depth]);
         
            // Cut half off
            translate([0,-(radius+1),-.5])
                cube ([radius+1,(radius+1)*2,depth+1]);
         
            // Cover the other half as necessary
            rotate([0,0,180-degrees])
            translate([0,-(radius+1),-.5])
                cube ([radius+1,(radius+1)*2,depth+1]);
         
        }
    }
}

// runde Ecken
module fillet(r, h) {
    translate([r / 2, r / 2, 0])

        difference() {
            cube([r + 0.01, r + 0.01, h], center = true);

            translate([r/2, r/2, 0])
                cylinder(r = r, h = h + 1, center = true);

        }
}

// Grundplatte
module plate(r1, r2, h){    
  translate([0,-42,0]) cylinder(h = h, r=r2, center = false);
  translate([0,0,0]) cylinder(h = h, r=r1, center = false);  
}


// Drehteller
module disc(){
  // Lidar-Drehteller Außen  
  translate([0,0,bear_z+2]) ring(bear_r2+1, bear_r2-3, 1);
  if (drawDiscTop)
    translate([0,0,bear_z+3]) ring(enc_r+2, slip_r4, 2);
  // Lidar-Drehteller Lager Innen
  translate([0,0,bear_z-5]) ring(bear_r2, slip_r1+0.5, bear_z);
  // Führung Rundring
  translate([0,0,bear_z+1]) ring_disc(disc_r, enc_r+2, 4, ring_thick);  
  //translate([0,0,bear_z+5]) ring(enc_r+3, enc_r+2, ring_thick);    
}

// Drehteller-Deckel
module disc_cover(){
  if (drawDiscCoverTop){
    difference(){
      difference(){
        cylinder(h=cover_h, r=cover_r, center=false);    
        translate([0,0,-1]) cylinder(h=cover_h, r=cover_r-1, center=false);          
      }    
      // Öffnungen für Lidar
      translate([(lidar_cyl_x/2-lidar_cyl_r),-10,16]) rotate([90,0,0])  cylinder(h = 50, r=lidar_cyl_r, center = false);
      translate([-(lidar_cyl_x/2-lidar_cyl_r),-10,16]) rotate([90,0, 0]) cylinder(h = 50, r=lidar_cyl_r, center = false);   
    }
  } 
  // Öffnungs-Ringe
  intersection(){        
    translate([(lidar_cyl_x/2-lidar_cyl_r),-15,16]) rotate([90,0,0]) ring(lidar_cyl_r+2, lidar_cyl_r+1, 20);    
    cylinder(h=cover_h, r=cover_r, center=false);    
  } 
  intersection(){        
    translate([-(lidar_cyl_x/2-lidar_cyl_r),-15,16]) rotate([90,0,0]) ring(lidar_cyl_r+2, lidar_cyl_r+1, 20);      
    cylinder(h=cover_h, r=cover_r, center=false);    
  }   
}    

// Lidar
module lidar(){
  translate([(lidar_cyl_x/2-lidar_cyl_r),0,0]) rotate([90,0,0]) cylinder(h = lidar_cyl_y, r=lidar_cyl_r, center = false);
  translate([-(lidar_cyl_x/2-lidar_cyl_r),0,0]) rotate([90,0,0]) cylinder(h = lidar_cyl_y, r=lidar_cyl_r, center = false);    
  translate([-lidar_box_x/2,0,-lidar_box_z/2]) cube([lidar_box_x, lidar_box_y, lidar_box_z], center = false);
  translate([-lidar_back_x/2,12,-lidar_back_z/2]) cube([lidar_back_x, lidar_back_y, lidar_back_z], center = false);
}


// Encoder-Pits
module encoder_pits(){
  ring(enc_r+2, enc_r, 1);  
  ring(enc_r, enc_r-1, 8-enc_z);  
  for ( i = [0 : 24 : 360] ){    
    a = (i < 25) ? 6 : 12;
    translate([0,0,8-enc_z]) rotate( i, [0, 0, 1]) arc( 1, enc_z, enc_r, a );  
  }
}

// Auflage für Schleifen-Kontakt
module slip_base(){  
  translate([0,0,bear_z1]) difference(){
    cylinder(h = 6, r1 = 8, r2 = 4, center = false);
    translate([0,0,-1]) cylinder(h = 6, r1 = 8, r2 = 4, center = false);    
    cylinder(h = 20, r=slip_r3, center = false, $fn=6);
  }   

  // Befestigung Auflagefläche für Schleifen-Kontakt
  translate([0,0,bear_z1]) ring(10, 7, 1);  
  for (i = [0:90:360]){
    rotate([0,0,i]) translate([-3,9,bear_z1]) cube([5, 14, 1], center = false);
  }
}

// Lager-Gehäuse (Ring)
module bearing_base(){  
  translate([0,0,bear_z1]) ring(enc_r+2, enc_r-4, 3);  
  translate([0,0,bear_z1]) ring(enc_r-1, enc_r-4, bear_z1+11);
  if (drawBearingRing) translate([0,0,bear_z1]) ring(bear_r1+2.5, bear_r1+1, bear_z+4);
  // Auflagefläche für Lager (Boden)
  translate([0,0,bear_z1+1]) ring(bear_r1+1, bear_r1-1.5, 3);
  translate([0,0,bear_z1]) ring(bear_r1+1, bear_r1-3.5, 3);
  for ( i = [0 : 24 : 360] ){    
    translate([sin(i)*(bear_r1+1), cos(i)*(bear_r1+1), 0]) translate([0,0,bear_z1]) cylinder(r=1, h=bear_z+3.5);  
  }  
}    

// Umrandung Grundplatte
module base_plate(){  
  translate([0,0,ground_z-4])
  difference(){  
    plate(35, 20, 1);
    translate([0,0,-1]) plate(34, 19, 3); 
  }  
  // Grundplatte und Öffnungen 
  difference(){    
    color("Tan", 1.0)   
    plate(35, 20, ground_z);
    translate([0,0,-10]) cylinder(r=bear_r1+6, h=20);  
    translate([0,-45,-1]) cylinder(r=motor_r, h=10); 
  }
  // Befestigungsstellen
  translate([-20,-44,-1]) cylinder(r=4, h=4);
  translate([+20,-44,-1]) cylinder(r=4, h=4);
  translate([+20,+30,-1]) cylinder(r=4, h=4);
  translate([-20,30,-1]) cylinder(r=4, h=4);  
}    

// Motorhalterung
module motor_base(){  
  translate([0,-45,0]) ring(motor_r+1, motor_r, motor_z); 
  difference(){
    translate([0,-45,motor_z]) cylinder(h = 1, r=motor_r+1, center = false);
    translate([0,-45,0]) cylinder(h = motor_z+10, r=5, center = false);
    translate([0,-53.0,0]) cylinder(h = motor_z+10, r=1, center = false);
    translate([0,-37.0,0]) cylinder(h = motor_z+10, r=1, center = false);
  }  
  //translate([-5,-32.5,0]) cube([10,4.5,6]);
}    


if (drawBase){
  // Grundplatte und Motorgehäuse
    if (drawBasePlate) color("LightBlue", 1.0) {
      base_plate();    
      motor_base();
  }     
  // Lagergehäuse
  if (drawBearingBase) color("LightGreen", 1.0) {    
    difference(){
      slip_base();
      // Öffnung für Schleifkontakt-Kabel (Stecker)
      rotate([0,0,45]) translate([-1,-1,-5])  cube([20,2,8]);
    }      
    bearing_base();
  }
  // Encoder
  if (drawEncoder) translate([0,0,-1]) encoder_pits();       
  // Lager
  if (drawBearing) {
    color("Red", 0.8) 
    translate([0,0,bear_z1+4]) ring(bear_r1, bear_r2, bear_z); 
    //color("Red", 0.5) 
    ///translate([-10,0,bear_z1+6]) ring(10, 5, 4); 
  }    
  // Motor-Rad
  if (drawMotorRing){
    translate([0,-45,motor_z+2]) ring_disc(motor_ring_r1, motor_ring_r2, 4, motor_ring_thick);  
  }
} // if (drawBase)


// Disc
if (drawDisc){
  color("LightBlue", 1.0)  
  difference(){
    disc();
    // Öffnung für Gabellichtschranke
    translate([-2,-enc_r-2,8]) cube([5,6,20]);
    // Klemmen für Schrauben
    for (i = [0:90:360]){
      rotate([0,0,i]) translate([-2,16,0]) cube([0.3, 4, 8], center = false);
      rotate([0,0,i]) translate([2,16,0]) cube([0.3, 4, 8], center = false);
      rotate([0,0,i]) translate([-2,16,0]) cube([4, 0.3, 8], center = false);
    }
  }    
}

if (drawSlipRing){
  #translate([0,0,bear_z+2]) ring(slip_r1, 5, 1);  
}    

// Lidar
if (drawLidar){
  translate([0,8, 29]) lidar();
}

// Deckel
if (drawDiscCover){
  color("Red", 0.5)  
  translate([0,0, 13]) disc_cover();    
}    

/*$fn=50;
minkowski()
{
 cube([10,10,1]);
 cylinder(r=2,h=1);
}

translate([0,0,-25]) minkowski();*/

