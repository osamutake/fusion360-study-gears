from dataclasses import dataclass, field


@dataclass
class Details:
    details = "Details"
    pressure_angle = "Pressure Angle"
    backlash = "Backlash"
    shift = "Shift"
    fillet = "Fillet"
    addendum = "Addendum"
    dedendum = "Dedendum"
    pressure_angle_tooltip = "The pressure angle of the gear (default = 20 deg)."
    backlash_tooltip = "Rotational/translational backlash between gears."
    shift_tooltip = "The shift with module as the unit."
    fillet_tooltip = "The maximum fillet radius of the cutting tool tip with module as the unit."
    addendum_tooltip = (
        "Distance from the pitch circle to the tip circle with module as the unit (typical: 1.0)."
    )
    dedendum_tooltip = "Distance from the pitch circle to the root circle with module as the unit (typical: 1.25)."
    r_clearance = "Clearance"
    r_clearance_tooltip = "The clearance between the root circle to the tip of the other gear with module as the unit (typical: 0.25), which determines the maximum fillet diameter."
    tip_fillet = "Tip fillet length"
    tip_fillet_tooltip = "Tip fillet extruding length from the tip circle with module as the unit. Only for hobbing."
    show_document = "Help"


@dataclass
class Cylindrical:
    cylindrical = "Cylindrical"
    module = "Module"
    module_tooltip = "The module of a gear is the teeth pitch length divided by PI."
    number_teeth = "Number of teeth"
    thickness = "Thickness"
    helix_angle = "Helix angle"
    helix_direction = "Helix direction"
    right = "Right"
    left = "Left"
    diameter = "Hole/Outer Diameter"
    diameter_tooltip = (
        "The outer diameter if Internal Gear is checked, otherwise the hole diameter."
    )
    internal = "Internal gear"
    worm_wheel = "Worm wheel"
    worm_diameter = "Worm diameter"
    worm_spirals = "Worm spirals"
    dp = "Reference diameter"
    dp_tooltip = "The reference diameter of the gear."


@dataclass
class RackWorm:
    rack_worm = "Rack/Worm"
    module = "Module"
    module_tooltip = "The module of a gear is the teeth pitch length divided by PI."
    thickness = "Thickness"
    thickness_tooltip = "The thickness of the rack gear or the diameter of the worm gear."
    length = "Length"
    helix_angle = "Helix angle"
    helix_direction = "Helix direction"
    right = "Right"
    left = "Left"
    worm_spirals = "Worm spirals"
    worm_spirals_tooltip = "The number of spiral turns for the worm gear. Set 0 for rack gear."
    height = "Height"
    height_tooltip = "The height of the rack gear."


@dataclass
class Bevel:
    bevel = "Bevel"
    axes_angle = "Axes angle"
    axes_angle_tooltip = "The angle between the axes of the two gears."
    module = "Module"
    module_tooltip = "The module of a gear is the teeth pitch length divided by PI."
    n_teeth1 = "Num. teeth 1"
    n_teeth2 = "Num. teeth 2"
    width = "Tooth width"
    spiral_angle = "Spiral angle"
    sphere_radius = "Sphere radius"
    sphere_radius_tooltip = "Radius of the virtual sphere that the bevel gear is inscribed in."
    gamma1 = "Angular radius 1"
    gamma1_tooltip = "The angular radius of gear1's pitch circle."
    gamma2 = "Angular radius 2"
    gamma2_tooltip = "The angular radius of gear2's pitch circle."
    printable = "Printable"
    printable_tooltip = "Make the gears 3D printer friendly."


@dataclass
class Crown:
    crown = "Crown"
    module = "Module"
    module_tooltip = "The module of a gear is the teeth pitch length divided by PI."
    crown_teeth = "Crown teeth"
    pinion_teeth = "Pinion teeth"
    helix_angle = "Helix angle"
    helix_direction = "Helix direction"
    right = "Right"
    left = "Left"
    outer_ext = "Outer extent"
    inner_ext = "Inner extent"
    outer_ext_tooltip = (
        "The outward width of the crown teeth from the reference circle "
        + "with module as the unit."
    )
    inner_ext_tooltip = (
        "The inward width of the crown teeth from the reference circle "
        + "with module as the unit."
    )


@dataclass
class Spiral:
    spiral = "Spiral"
    angle = "Total Angle"
    angle_tooltip = "The total turning angle of the spiral curve."
    radii = "Radii"
    radii_tooltip = (
        "The radii of the spiral curve. Separate multiple values with commas,\n"
        + "The specified radii are evenly spaced within the total angle \n"
        + "and interpolated to create a smooth spiral."
    )
    height = "Height"
    height_tooltip = "The height of the spiral curve. Set 0 for a flat spiral."
    flip = "Flip"
    flip_tooltip = "Flip the spiral turn direction."
    spline = "Spline"
    spline_tooltip = (
        "Use spline interpolation instead of linear interpolation between radii, \n"
        + "which affects only when more than 5 radii are given and \n"
        + "the first and last values are equal to each other.\n"
        + "This is for generating the shape of a cam."
    )


@dataclass
class Locale:
    details: Details = field(default_factory=Details)
    cylindrical: Cylindrical = field(default_factory=Cylindrical)
    rack_worm: RackWorm = field(default_factory=RackWorm)
    bevel: Bevel = field(default_factory=Bevel)
    crown: Crown = field(default_factory=Crown)
    spiral: Spiral = field(default_factory=Spiral)


LOCALE = Locale()
