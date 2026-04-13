difference()
{
    cube([34,10,20]);
    translate([5,0,12])cube([24,10,8]);  
    translate([17,10,0])rotate([90,0,0])cylinder(d=16.2 ,h=20);
    #translate([0,2.5,8])cube([34,5,2]);  
}