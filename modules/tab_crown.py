from typing import override
from math import cos, atan, tan
import adsk.core, adsk.fusion

from . import gear_curve
from . import command
from .gear_crown import gear_crown
from .lib import fusion_helper as fh
from .locales import LOCALE

l = LOCALE.crown


class TabInput(fh.TabInput[command.Command]):
    id = "crown_tab"
    name = l.crown

    # parent: command.Command
    # tab: adsk.core.TabCommandInput

    module: adsk.core.ValueCommandInput
    pinion_teeth: adsk.core.IntegerSpinnerCommandInput
    number_teeth: adsk.core.IntegerSpinnerCommandInput
    helix_angle: adsk.core.ValueCommandInput
    helix_direction: adsk.core.RadioButtonGroupCommandInput

    @override
    def on_created(self, args: adsk.core.CommandCreatedEventArgs, inputs: adsk.core.CommandInputs):
        self.module = fh.value_control(inputs, "module", l.module, "mm", "4", min_exclusive=0)
        self.module.tooltip = l.module_tooltip
        self.number_teeth = inputs.addIntegerSpinnerCommandInput(
            "number_teeth", l.crown_teeth, 6, 200, 1, 40
        )
        self.pinion_teeth = inputs.addIntegerSpinnerCommandInput(
            "pinion_teeth", l.pinion_teeth, 6, 200, 1, 12
        )

        self.width1 = fh.value_control(inputs, "width1", l.outer_ext, "", "2", min=0)
        self.width2 = fh.value_control(inputs, "width2", "Inner Extent", "", "2", min=0)
        self.width1.tooltip = l.outer_ext_tooltip
        self.width2.tooltip = l.inner_ext_tooltip
        self.helix_angle = fh.value_control(
            inputs, "helix_angle", l.helix_angle, "deg", "0", min=-90, max=90
        )
        self.helix_direction = inputs.addRadioButtonGroupCommandInput(
            "helix_direction",
            l.helix_direction,
        )
        self.helix_direction.listItems.add(l.right, True)
        self.helix_direction.listItems.add(l.left, False)

    @override
    def on_changed(self, args: adsk.core.InputChangedEventArgs | None):
        pass

    @override
    def on_execute_or_preview(self, _: adsk.core.CommandEventArgs, is_preview: bool = False):
        if is_preview:
            return

        helix_angle = self.helix_angle.value
        if self.helix_direction.selectedItem.name == l.left:
            helix_angle *= -1

        # parameters for the pinion
        params = gear_curve.GearParams(
            m=self.module.value / cos(helix_angle),
            z=self.pinion_teeth.value,
            alpha=atan(tan(self.parent.pressure_angle.value) / cos(helix_angle)),
            shift=self.parent.shift.value,
            fillet=0.4,
            mf=self.parent.dedendum.value * cos(helix_angle),
            mk=self.parent.addendum.value * cos(helix_angle),
            rc=(self.parent.dedendum.value - self.parent.addendum.value) * cos(helix_angle),
            backlash=-self.parent.backlash.value,
            inner=False,
        )

        generate_gear(
            params, self.number_teeth.value, self.width1.value, self.width2.value, helix_angle
        )


def generate_gear(
    params: gear_curve.GearParams,
    z: int,
    w1: float,
    w2: float,
    helix_angle: float,
):
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)

    # Create the wrapper component
    wrapper_occurrence = design.activeComponent.occurrences.addNewComponent(
        adsk.core.Matrix3D.create()
    )
    if design.activeOccurrence is not None:
        wrapper_occurrence = wrapper_occurrence.createForAssemblyContext(design.activeOccurrence)
    wrapper_occurrence.isGroundToParent = False
    wrapper = wrapper_occurrence.component

    # generate sub component containing the gear
    gear = gear_crown(wrapper_occurrence, params, z, w1, w2, helix_angle)
    gear.name = f"crown{format(params.m*10,".2g")}M{z}T"
    wrapper.name = gear.name[0:1].capitalize() + gear.name[1:]
