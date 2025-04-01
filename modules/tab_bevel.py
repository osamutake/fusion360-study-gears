from typing import override

import adsk.core, adsk.fusion

from . import command
from .lib import fusion_helper as fh
from .gear_bevel import gear_bevel


class TabInput(fh.TabInput[command.Command]):
    id = "bevel_tab"
    name = "Bevel"

    # parent: command.Command
    # tab: adsk.core.TabCommandInput

    axes_angle: adsk.core.ValueCommandInput
    module: adsk.core.ValueCommandInput
    z1: adsk.core.IntegerSpinnerCommandInput
    z2: adsk.core.IntegerSpinnerCommandInput
    pressure_angle: adsk.core.ValueCommandInput
    width: adsk.core.ValueCommandInput
    beta: adsk.core.ValueCommandInput

    @override
    def on_created(self, _: adsk.core.CommandCreatedEventArgs, inputs: adsk.core.CommandInputs):
        self.axes_angle = fh.value_control(inputs, "bevel_axes_angle", "Axes Angle", "deg", "90")
        self.module = fh.value_control(
            inputs, "bevel_module", "Module", "mm", "3", min_exclusive=0
        )
        self.z1 = inputs.addIntegerSpinnerCommandInput("bevel_z1", "Num. Teeth 1", 6, 200, 1, 20)
        self.z2 = inputs.addIntegerSpinnerCommandInput("bevel_z2", "Num. Teeth 2", 6, 200, 1, 40)
        self.width = fh.value_control(inputs, "bevel_width", "Width", "mm", "10")
        self.beta = fh.value_control(inputs, "bevel_beta", "Helical Angle", "deg", "0")

    @override
    def on_execute_or_preview(self, args: adsk.core.CommandEventArgs, is_preview: bool = False):
        if is_preview:
            return

        # かさば歯車を生成する
        return gear_bevel(
            self.axes_angle.value,
            self.module.value,
            self.z1.value,
            self.z2.value,
            self.width.value,
            self.beta.value,
            self.parent.addendum.value,
            self.parent.dedendum.value,
            self.parent.pressure_angle.value,
            self.parent.backlash.value,
        )
