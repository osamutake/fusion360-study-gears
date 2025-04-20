from math import asin
from typing import override

import adsk.core, adsk.fusion

from .lib import fusion_helper as fh
from . import command
from .gear_rack import RackParams, gear_rack
from .gear_worm import gear_worm
from .locales import LOCALE

EPS = 1e-5

l = LOCALE.rack_worm


class TabInput(fh.TabInput[command.Command]):
    id = "rack_tab"
    name = l.rack_worm

    # parent: command.Command
    # tab: adsk.core.TabCommandInput

    module: adsk.core.ValueCommandInput
    thickness: adsk.core.ValueCommandInput
    height: adsk.core.ValueCommandInput
    length: adsk.core.ValueCommandInput
    angle: adsk.core.ValueCommandInput
    worm: adsk.core.IntegerSpinnerCommandInput
    worm_previous: int = 0
    worm_direction: adsk.core.RadioButtonGroupCommandInput
    angle_previous: float = 0

    @override
    def on_created(self, _: adsk.core.CommandCreatedEventArgs, inputs: adsk.core.CommandInputs):
        self.module = fh.value_control(inputs, "rack_module", l.module, "mm", "4", min_exclusive=0)
        self.module.tooltip = l.module_tooltip
        self.thickness = fh.value_control(
            inputs, "rack_thickness", l.thickness, "mm", "15", min_exclusive=0
        )
        self.thickness.tooltip = l.thickness_tooltip
        self.length = fh.value_control(
            inputs, "rack_length", l.length, "mm", "200", min_exclusive=0
        )
        self.angle = fh.value_control(
            inputs, "rack_angle", l.helix_angle, "deg", "0", min=-60, max=60
        )
        self.worm_direction = inputs.addRadioButtonGroupCommandInput(
            "angle_direction", l.helix_direction
        )
        self.worm_direction.listItems.add(l.right, True)
        self.worm_direction.listItems.add(l.left, False)
        self.worm = inputs.addIntegerSpinnerCommandInput("rack_worm", l.worm_spirals, 0, 10, 1, 0)
        self.worm.tooltip = l.worm_spirals_tooltip
        self.height = fh.value_control(
            inputs, "rack_height", l.height, "mm", "20", min_exclusive=0
        )
        self.height.tooltip = l.height_tooltip

    @override
    def on_changed(self, args: adsk.core.InputChangedEventArgs | None):
        if not self.tab.isActive:
            return

        # 歯先フィレットに対応
        self.parent.tip_fillet.isEnabled = True
        # 「高さ」はラック作成時のみ有効
        self.height.isVisible = self.worm.value == 0
        # 条数が0でなければウォームを生成するため傾斜角は計算で求める
        self.angle.isEnabled = self.worm.value == 0
        if not self.angle.isEnabled and self.thickness.value > 0:
            self.angle.value = asin(self.worm.value * self.module.value / self.thickness.value)

        # ウォームの条数変更
        if args and args.input.id == "rack_worm":
            # ウォームの直径はラックの厚さの倍にする
            if self.worm.value == 0:
                self.thickness.value /= 2
            elif self.worm_previous == 0:
                self.thickness.value *= 2
                self.angle.value = self.angle_previous

        self.worm_previous = self.worm.value
        if self.worm.value == 0:
            self.angle_previous = self.angle.value

    @override
    def on_execute_or_preview(self, _: adsk.core.CommandEventArgs, is_preview: bool = False):
        if is_preview:
            return

        params = RackParams(
            m=self.module.value,
            thickness=self.thickness.value,
            height=self.height.value,
            length=self.length.value,
            angle=self.angle.value,
            worm=self.worm.value,
            mk=self.parent.addendum.value,
            mf=self.parent.dedendum.value,
            rc=self.parent.r_clearance.value,
            alpha=self.parent.pressure_angle.value,
            backlash=self.parent.backlash.value,
            fillet=self.parent.fillet.value,
        )
        if params.worm > 0:
            return gear_worm(
                params,
                self.parent.tip_fillet.value,
                1 if self.worm_direction.selectedItem.name == l.right else -1,
            )
        else:
            if self.worm_direction.selectedItem.name == l.left:
                params.angle *= -1
            return gear_rack(params, self.parent.tip_fillet.value)
