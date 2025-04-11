from math import asin, cos, pi
from typing import override
import adsk.core, adsk.fusion

from . import gear_curve
from . import command
from .gear_worm_wheel import gear_worm_wheel
from .gear_cylindrical import gear_cylindrical
from .lib import fusion_helper as fh
from .locales import LOCALE

l = LOCALE.cylindrical


class TabInput(fh.TabInput[command.Command]):
    id = "cylindrical_tab"
    name = l.cylindrical

    # parent: command.Command
    # tab: adsk.core.TabCommandInput

    module: adsk.core.ValueCommandInput
    number_teeth: adsk.core.IntegerSpinnerCommandInput
    thickness: adsk.core.ValueCommandInput
    worm_diameter: adsk.core.ValueCommandInput
    worm_wheel: adsk.core.BoolValueCommandInput
    worm_spirals: adsk.core.IntegerSpinnerCommandInput
    helix_direction: adsk.core.RadioButtonGroupCommandInput
    internal: adsk.core.BoolValueCommandInput
    helix_angle: adsk.core.ValueCommandInput
    dp: adsk.core.ValueCommandInput
    diameter: adsk.core.ValueCommandInput

    @override
    def on_created(self, args: adsk.core.CommandCreatedEventArgs, inputs: adsk.core.CommandInputs):
        self.module = fh.value_control(inputs, "module", l.module, "mm", "4", min_exclusive=0)
        self.module.tooltip = l.module_tooltip

        self.number_teeth = inputs.addIntegerSpinnerCommandInput(
            "number_teeth", l.number_teeth, 6, 200, 1, 12
        )
        self.thickness = fh.value_control(inputs, "thickness", l.thickness, "", "5")
        self.helix_angle = fh.value_control(
            inputs, "helix_angle", l.helix_angle, "deg", "0", min=-60, max=60
        )
        self.helix_direction = inputs.addRadioButtonGroupCommandInput(
            "helix_direction", l.helix_direction
        )
        self.helix_direction.listItems.add(l.right, True)
        self.helix_direction.listItems.add(l.left, False)
        self.diameter = fh.value_control(inputs, "diameter", l.diameter, "mm", "0", min=0)
        self.diameter.tooltip = l.diameter_tooltip
        self.internal = inputs.addBoolValueInput("internal", l.internal, True, "", False)
        self.worm_wheel = inputs.addBoolValueInput("worm_wheel", l.worm_wheel, True, "", False)
        self.worm_diameter = fh.value_control(
            inputs, "worm_diameter", l.worm_diameter, "mm", "0", min=0, is_visible=False
        )
        self.worm_spirals = inputs.addIntegerSpinnerCommandInput(
            "worm_spirals", l.worm_spirals, 1, 10, 1, 1
        )
        self.worm_spirals.isVisible = False
        self.dp = fh.value_control(inputs, "dp", l.dp, "mm", "48", is_enabled=False)
        self.dp.tooltip = l.dp_tooltip

    @override
    def on_changed(self, args: adsk.core.InputChangedEventArgs | None):
        self.thickness.isVisible = self.tab.isActive

        if not self.tab.isActive:
            self.worm_diameter.isMinimumLimited = False
            self.worm_diameter.isMaximumLimited = False
            return

        worm_wheel = self.worm_wheel.value
        internal = self.internal.value
        module = self.module.value
        worm_diameter = self.worm_diameter.value
        worm_spirals = self.worm_spirals.value
        helix_angle = self.helix_angle.value
        number_teeth = self.number_teeth.value
        dp = self.dp.value

        self.helix_angle.isEnabled = not worm_wheel
        self.worm_diameter.isVisible = worm_wheel
        self.worm_spirals.isVisible = worm_wheel
        self.internal.isVisible = not worm_wheel
        self.worm_wheel.isVisible = not internal
        if worm_wheel:
            self.internal.value = False
            internal = False
            self.worm_diameter.minimumValue = module * 4
            self.worm_diameter.isMinimumLimited = True
            if worm_diameter >= self.worm_diameter.minimumValue:
                self.helix_angle.value = asin(worm_spirals * module / worm_diameter)
        elif args and args.input.id == "worm_wheel":
            self.helix_angle.value = 0

        if internal:
            self.worm_wheel.value = False

        self.parent.tip_fillet.isEnabled |= (
            self.tab.isActive and not self.worm_wheel.value and not self.internal.value
        )

        # calculate reference diameter
        self.dp.value = module / cos(helix_angle) * number_teeth

        # max/min values for diameter
        if internal:
            self.diameter.minimumValue = (
                dp + module * (self.parent.shift.value - self.parent.addendum.value) * 2
            )
            self.diameter.isMinimumLimited = True
            self.diameter.isMinimumInclusive = False
            self.diameter.isMaximumLimited = False
        else:
            self.diameter.maximumValue = (
                dp + module * (self.parent.shift.value - self.parent.addendum.value) * 2
            )
            self.diameter.isMinimumLimited = False
            self.diameter.isMaximumLimited = False
            self.diameter.isMaximumInclusive = False

    @override
    def on_execute_or_preview(self, _: adsk.core.CommandEventArgs, is_preview: bool = False):
        if is_preview:
            return

        params = gear_curve.GearParams(
            m=self.module.value,
            z=self.number_teeth.value,
            alpha=self.parent.pressure_angle.value,
            shift=self.parent.shift.value,
            fillet=self.parent.fillet.value,
            mf=self.parent.dedendum.value,
            mk=self.parent.addendum.value,
            rc=self.parent.r_clearance.value,
            backlash=self.parent.backlash.value,
            inner=self.internal.value,
        )

        worm_diameter = self.worm_diameter.value
        worm_spirals = self.worm_spirals.value
        if not self.worm_wheel.value:
            worm_diameter = 0
            worm_spirals = 0
        if self.helix_direction.selectedItem.name == "Left":
            self.helix_angle.value = -self.helix_angle.value

        generate_gear(
            params,
            self.thickness.value,
            self.helix_angle.value,
            self.diameter.value,
            worm_diameter,
            worm_spirals,
            self.internal.value,
            self.parent.tip_fillet.value,
        )


def generate_gear(
    params: gear_curve.GearParams,
    thickness: float,
    helix_angle: float,
    diameter: float,
    worm_diameter: float,
    worm_spirals: int,
    internal: bool,
    tip_fillet: float,
):
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)

    # wrapper component
    wrapper_occurrence = design.activeComponent.occurrences.addNewComponent(
        adsk.core.Matrix3D.create()
    )
    wrapper_occurrence.isGroundToParent = False
    wrapper = wrapper_occurrence.component

    # generate the gear
    if worm_diameter > 0:
        gear_worm_wheel(
            wrapper_occurrence, params, worm_diameter, worm_spirals, thickness, helix_angle
        )
    else:
        gear_cylindrical(
            wrapper,
            params,
            thickness,
            helix_angle,
            tip_fillet,
        )

    gear = wrapper.occurrences.item(0).component
    wrapper.occurrences.item(0).isGroundToParent = False
    if worm_diameter > 0:
        gear.name = f"wheel{format(params.m*10,".2g")}M{params.z}T"
    elif internal:
        gear.name = f"internal{format(params.m*10,".2g")}M{params.z}T"
    elif helix_angle != 0:
        gear.name = f"helical{format(params.m*10,".2g")}M{params.z}T"
    else:
        gear.name = f"spur{format(params.m*10,".2g")}M{params.z}T"
    if params.shift != 0:
        gear.name += f"_{format(params.shift,".2g")}S"

    wrapper.name = gear.name[0:1].capitalize() + gear.name[1:]

    # create the central hole or outer disk of internal gear
    rp = params.m / cos(helix_angle) * params.z
    if (internal and diameter > rp) or (not internal and 0 < diameter < rp - params.m * params.mf):
        # generate disk
        outer = gear.sketches.add(
            gear.xYConstructionPlane
        ).sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), diameter / 2
        )
        outer.isFixed = True

        profiles = gear.sketches.item(gear.sketches.count - 1).profiles.item(0)
        fh.comp_extrude(gear, profiles, fh.FeatureOperations.new_body, thickness, True, True)

        # compare volumes of the disk and the gear
        b0 = gear.bRepBodies.item(0)
        b1 = gear.bRepBodies.item(1)
        if b0.volume < b1.volume:
            b0, b1 = b1, b0

        # cut smaller part from the larger part
        fh.comp_combine(gear, b0, b1, fh.FeatureOperations.cut)

    # create a joint geometry
    fh.comp_joint_revolute(
        wrapper,
        gear.originConstructionPoint,
        wrapper.originConstructionPoint,
        adsk.fusion.JointDirections.ZAxisJointDirection,
    )

    # move worm wheel to the meshing position
    if worm_spirals > 0:
        matrix = fh.matrix_rotate(
            pi / 2, wrapper.xConstructionAxis.geometry.direction, fh.point3d()
        )
        matrix.translation = fh.vector3d(
            params.m * (params.z / cos(helix_angle) / 2 + params.shift) + worm_diameter / 2,
            0,
            pi * params.m / cos(helix_angle) * 2,
        )
        wrapper_occurrence.isGroundToParent = False
        design.activeComponent.transformOccurrences([wrapper_occurrence], [matrix], False)
        design.snapshots.add()  # capture the position
