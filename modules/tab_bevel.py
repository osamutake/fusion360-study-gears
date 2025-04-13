from math import atan, atan2, cos, pi, sin, tan
from typing import override

import adsk.core, adsk.fusion

from . import command
from .lib import fusion_helper as fh
from .gear_bevel import gear_bevel
from .locales import LOCALE

l = LOCALE.bevel


class TabInput(fh.TabInput[command.Command]):
    id = "bevel_tab"
    name = l.bevel

    # parent: command.Command
    # tab: adsk.core.TabCommandInput

    axes_angle: adsk.core.ValueCommandInput
    module: adsk.core.ValueCommandInput
    z1: adsk.core.IntegerSpinnerCommandInput
    z2: adsk.core.IntegerSpinnerCommandInput
    pressure_angle: adsk.core.ValueCommandInput
    width: adsk.core.ValueCommandInput
    beta: adsk.core.ValueCommandInput
    sphere_radius: adsk.core.ValueCommandInput
    gamma1: adsk.core.ValueCommandInput
    gamma2: adsk.core.ValueCommandInput

    @override
    def on_created(self, _: adsk.core.CommandCreatedEventArgs, inputs: adsk.core.CommandInputs):
        self.axes_angle = fh.value_control(inputs, "bevel_axes_angle", l.axes_angle, "deg", "90")
        self.axes_angle.tooltip = l.axes_angle_tooltip
        self.module = fh.value_control(
            inputs, "bevel_module", l.module, "mm", "3", min_exclusive=0
        )
        self.z1 = inputs.addIntegerSpinnerCommandInput("bevel_z1", l.n_teeth1, 6, 200, 1, 20)
        self.z2 = inputs.addIntegerSpinnerCommandInput("bevel_z2", l.n_teeth2, 6, 200, 1, 40)
        self.width = fh.value_control(inputs, "bevel_width", l.width, "mm", "10")
        self.beta = fh.value_control(inputs, "bevel_beta", l.spiral_angle, "deg", "0")

        self.sphere_radius = fh.value_control(
            inputs, "bevel_sphere_radius", l.sphere_radius, "mm", "0", is_enabled=False
        )
        self.sphere_radius.tooltip = l.sphere_radius_tooltip
        self.gamma1 = fh.value_control(
            inputs, "bevel_gamma1", l.gamma1, "deg", "0", is_enabled=False
        )
        self.gamma1.tooltip = l.gamma1_tooltip
        self.gamma2 = fh.value_control(
            inputs, "bevel_gamma2", l.gamma2, "deg", "0", is_enabled=False
        )
        self.gamma2.tooltip = l.gamma2_tooltip

    @override
    def on_changed(self, args: adsk.core.InputChangedEventArgs | None):
        if self.tab.isActive:
            self.parent.fillet.isEnabled = False
            self.parent.shift.isEnabled = False
            self.parent.r_clearance.isEnabled = False

            m = self.module.value
            sigma = self.axes_angle.value
            alpha = self.parent.pressure_angle.value
            beta = self.beta.value
            z1 = self.z1.value
            z2 = self.z2.value
            m *= 1 / cos(beta)
            alpha = atan(tan(alpha) / cos(beta))

            # basic dimensions
            gamma_p1 = atan2(sin(sigma), z2 / z1 + cos(sigma))
            gamma_p2 = sigma - gamma_p1

            r = z1 / (2 * sin(gamma_p1))
            r0 = r * m

            self.sphere_radius.value = r0
            self.gamma1.value = gamma_p1
            self.gamma2.value = gamma_p2

    @override
    def on_execute_or_preview(self, args: adsk.core.CommandEventArgs, is_preview: bool = False):
        if is_preview:
            return

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
