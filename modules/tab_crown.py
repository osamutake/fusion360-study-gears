from typing import cast, override
import adsk.core, adsk.fusion

from . import gear_curve
from . import command
from .gear_crown import gear_crown
from .lib import fusion_helper as fh


class TabInput(fh.TabInput[command.Command]):
    id = "crown_tab"
    name = "Crown"

    # parent: command.Command
    # tab: adsk.core.TabCommandInput

    module: adsk.core.ValueCommandInput
    pinion_teeth: adsk.core.IntegerSpinnerCommandInput
    number_teeth: adsk.core.IntegerSpinnerCommandInput

    @override
    def on_created(self, args: adsk.core.CommandCreatedEventArgs, inputs: adsk.core.CommandInputs):
        self.module = fh.value_control(inputs, "module", "Module", "mm", "4", min_exclusive=0)
        self.module.tooltip = "The module of a gear is the teeth pitch length divided by PI."
        self.number_teeth = inputs.addIntegerSpinnerCommandInput(
            "number_teeth", "Crown Teeth", 6, 200, 1, 40
        )
        self.pinion_teeth = inputs.addIntegerSpinnerCommandInput(
            "pinion_teeth", "Pinion Teeth", 6, 200, 1, 12
        )

        self.width1 = fh.value_control(inputs, "width1", "Outer Extent", "", "2", min=0)
        self.width2 = fh.value_control(inputs, "width2", "Inner Extent", "", "2", min=0)
        self.width1.tooltip = (
            "Outward width of crown teeth from reference circle with module as unit."
        )
        self.width2.tooltip = (
            "Inward width of crown teeth from reference circle with module as unit."
        )

    @override
    def on_changed(self, args: adsk.core.InputChangedEventArgs | None):
        pass

    @override
    def on_execute_or_preview(self, _: adsk.core.CommandEventArgs, is_preview: bool = False):
        if is_preview:
            return

        # parameters for the pinion
        params = gear_curve.GearParams(
            m=self.module.value,
            z=self.pinion_teeth.value,
            alpha=self.parent.pressure_angle.value,
            shift=self.parent.shift.value,
            fillet=0.4,
            mf=self.parent.dedendum.value,
            mk=self.parent.addendum.value,
            rc=self.parent.dedendum.value - self.parent.addendum.value,
            backlash=-self.parent.backlash.value,
            inner=False,
        )

        generate_gear(
            params,
            self.number_teeth.value,
            self.width1.value,
            self.width2.value,
        )


def generate_gear(
    params: gear_curve.GearParams,
    z: int,
    w1: float,
    w2: float,
):
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)

    # ラッパーコンポーネントを作成
    comp_occurrence = design.activeComponent.occurrences.addNewComponent(
        adsk.core.Matrix3D.create()
    )
    comp_occurrence.isGroundToParent = False
    comp = comp_occurrence.component

    # generate sub component containing the gear
    gear_crown(comp, params, z, w1, w2)

    gear = comp.occurrences.item(0).component
    comp.occurrences.item(0).isGroundToParent = False
    gear.name = f"crown{format(params.m*10,".2g")}M{z}T"
    comp.name = gear.name[0:1].capitalize() + gear.name[1:]

    # create a joint between the wrapper and the gear components
    fh.comp_built_joint_revolute(
        comp,
        comp.occurrences[0].createForAssemblyContext(comp_occurrence),
        comp_occurrence,
        comp.occurrences[0].component.sketches[0].profiles[0],
        cast(adsk.fusion.JointDirections, adsk.fusion.JointDirections.ZAxisJointDirection),
    )
