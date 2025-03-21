from math import asin, cos, pi
from typing import override
import adsk.core, adsk.fusion

from . import gear_curve
from . import command
from .gear_worm_wheel import gear_worm_wheel
from .gear_cylindrical import gear_cylindrical
from .lib import fusion_helper as fh


class TabInput(fh.TabInput[command.Command]):
    id = "cylindrical_tab"
    name = "Cylindrical"

    # parent: command.Command
    # tab: adsk.core.TabCommandInput

    module: adsk.core.ValueCommandInput
    number_teeth: adsk.core.IntegerSpinnerCommandInput
    thickness: adsk.core.DistanceValueCommandInput
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
        # 平歯車のパラメータ
        self.module = fh.value_control(inputs, "module", "Module", "mm", "4", min_exclusive=0)
        self.module.tooltip = "The module of a gear is the teeth pitch length divided by PI."

        self.number_teeth = inputs.addIntegerSpinnerCommandInput(
            "number_teeth", "Num. Teeth", 6, 200, 1, 12
        )
        self.thickness = inputs.addDistanceValueCommandInput(
            "thickness", "Thickness", fh.value_input("5")
        )
        self.thickness.setManipulator(fh.point3d(), fh.vector3d(z=1))
        self.helix_angle = fh.value_control(
            inputs, "helix_angle", "Helix Angle", "deg", "0", min=-60, max=60
        )
        self.helix_direction = inputs.addRadioButtonGroupCommandInput(
            "helix_direction", "Helix Direction"
        )
        self.helix_direction.listItems.add("Right", True)
        self.helix_direction.listItems.add("Left", False)
        self.diameter = fh.value_control(
            inputs, "diameter", "Hole/Outer Diameter", "mm", "0", min=0
        )
        self.diameter.tooltip = (
            "The outer diameter if Internal Gear is checked, otherwise the hole diameter."
        )
        self.internal = inputs.addBoolValueInput("internal", "Internal Gear", True, "", False)
        self.worm_wheel = inputs.addBoolValueInput("worm_wheel", "Worm Wheel", True, "", False)
        self.worm_diameter = fh.value_control(
            inputs, "worm_diameter", "Worm Diameter", "mm", "0", min=0, is_visible=False
        )
        self.worm_spirals = inputs.addIntegerSpinnerCommandInput(
            "worm_spirals", "Num. Spiral", 1, 10, 1, 1
        )
        self.worm_spirals.isVisible = False
        self.dp = fh.value_control(
            inputs, "dp", "Reference Diameter", "mm", "48", is_enabled=False
        )

    @override
    def on_changed(self, args: adsk.core.InputChangedEventArgs | None):
        self.thickness.isVisible = self.tab.isActive

        if not self.tab.isActive:
            self.worm_diameter.isMinimumLimited = False
            self.worm_diameter.isMaximumLimited = False
            return

        # 変更への対応

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

        # 基準円直径を計算
        self.dp.value = module / cos(helix_angle) * number_teeth

        # 穴・外形の最小値・最大値を設定
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

        # 平歯車を生成する
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

    # ラッパーコンポーネントを作成
    comp_occurrence = design.activeComponent.occurrences.addNewComponent(
        adsk.core.Matrix3D.create()
    )
    comp_occurrence.isGroundToParent = False
    comp = comp_occurrence.component

    # 歯車コンポーネントを作成
    if worm_diameter > 0:
        gear_worm_wheel(comp, params, worm_diameter, worm_spirals, thickness, helix_angle)
    else:
        gear_cylindrical(
            comp,
            params,
            thickness,
            helix_angle,
            tip_fillet,
        )

    gear = comp.occurrences.item(0).component
    comp.occurrences.item(0).isGroundToParent = False
    if worm_diameter > 0:
        # pylint: disable=inconsistent-quotes
        gear.name = f"wheel{format(params.m*10,'.2g')}M{params.z}T"
    elif internal:
        # pylint: disable=inconsistent-quotes
        gear.name = f"internal{format(params.m*10,'.2g')}M{params.z}T"
    elif helix_angle != 0:
        # pylint: disable=inconsistent-quotes
        gear.name = f"helical{format(params.m*10,'.2g')}M{params.z}T"
    else:
        # pylint: disable=inconsistent-quotes
        gear.name = f"spur{format(params.m*10,'.2g')}M{params.z}T"
    if params.shift != 0:
        # pylint: disable=inconsistent-quotes
        gear.name += f"_{format(params.shift,'.2g')}S"

    comp.name = gear.name[0:1].capitalize() + gear.name[1:]

    # 中央の穴あるいは内歯車の外枠を生成する
    rp = params.m / cos(helix_angle) * params.z
    if (internal and diameter > rp) or (not internal and 0 < diameter < rp - params.m * params.mf):
        # 円筒を生成
        outer = gear.sketches.add(
            gear.xYConstructionPlane
        ).sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), diameter / 2
        )
        outer.isFixed = True

        profiles = gear.sketches.item(gear.sketches.count - 1).profiles.item(0)
        fh.comp_extrude(gear, profiles, fh.FeatureOperations.new_body, thickness, True, True)

        # 円筒と歯車の大きい方から小さい方を切り取る
        b0 = gear.bRepBodies.item(0)
        b1 = gear.bRepBodies.item(1)
        if b0.volume < b1.volume:
            b0, b1 = b1, b0
        fh.comp_combine(gear, b0, b1, fh.FeatureOperations.cut)

    # Create a joint geometry
    fh.comp_joint_revolute(
        comp,
        gear.originConstructionPoint,
        comp.originConstructionPoint,
        adsk.fusion.JointDirections.ZAxisJointDirection,
    )

    # ウォームホイールをウォームと噛み合う位置に移動する
    if worm_spirals > 0:
        matrix = fh.matrix_rotate(pi / 2, comp.xConstructionAxis.geometry.direction, fh.point3d())
        matrix.translation = fh.vector3d(
            params.m * (params.z / cos(helix_angle) / 2 + params.shift) + worm_diameter / 2,
            0,
            pi * params.m / cos(helix_angle) * 2,
        )
        comp_occurrence.isGroundToParent = False
        design.activeComponent.transformOccurrences([comp_occurrence], [matrix], False)
        design.snapshots.add()  # 位置をキャプチャ
