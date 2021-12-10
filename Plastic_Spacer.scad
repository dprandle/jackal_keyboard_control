$fa = 1;
$fs = 0.1;
inches = 25.4;

outer_d = 0.25 * inches;
inner_d = (1 / 8 + 0.005) * inches;


module cylinder_shaft(inner_diam, outer_diam, height)
{
    linear_extrude(height)
    {
        difference()
        {
            circle(d=outer_d);
            circle(d=inner_d);
        }
    }
}

cylinder_shaft(innder_d, outer_d, outer_d);