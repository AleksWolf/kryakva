"""Control law of the Feetech HD-1910-C001 for BAM.

Ported from JoyandAI/microduck_rl, branch develop, vendor/bam/bam/feetech/actuator.py,
class HD1910Actuator (Apache-2.0, based on BAM by Marc Duclusaud & Gregoire Passault), with their
default use_rate_limiting=False: register 33 = 4 (pure position PD), registers 84/85/86 = 0,
so the firmware tracks the goal directly and the law is stateless:

    duty = (q_target - q) * kp * error_gain + (-dq) * kd * error_gain / 4
    duty limited so that |I| = |duty * vin - kt * dq| / R <= max_current, then to +-max_pwm

kp = 32, kd = 40: servo registers 21/22; error_gain, kd and the friction model: JoyandAI's pendulum
fit (params in training/actuators/hd1910_bam_m5.json).

The installed BAM package has no "hd1910" actuator, so importing this module adds it
to bam.actuators.actuators; the package itself is not modified.
"""

from __future__ import annotations

import bam.actuators
from bam.actuator import VoltageControlledActuator
from bam.parameter import Parameter
from bam.testbench import Pendulum


class HD1910Actuator(VoltageControlledActuator):
    D_SCALE = 4.0  # D term scale 1/4 (HLS convention)

    def __init__(self, testbench_class=None):
        super().__init__(testbench_class, vin=7.4, kp=32, error_gain=0.163, max_pwm=1.0)

    def get_extra_inertia(self) -> float:
        return self.model.armature.value

    def initialize(self):
        self.model.kt = Parameter(0.7358, 0.1, 2.0)
        self.model.R = Parameter(3.75, 0.5, 10.0)
        self.model.armature = Parameter(1e-3, 1e-6, 0.05)
        self.model.error_gain = Parameter(self.error_gain, 0.01, 10.0)
        self.model.kd = Parameter(40.0, 0.0, 254.0)
        self.model.max_current = Parameter(3.25, 0.1, 5.0)

    def compute_control(self, q_target, q, dq, dt):
        error_gain = self.model.error_gain.value
        duty = (q_target - q) * self.kp * error_gain + (-dq) * self.model.kd.value * error_gain / self.D_SCALE
        if self.model.max_current.value > 0:
            center = self.model.kt.value * dq / self.vin
            span = self.model.R.value * self.model.max_current.value / self.vin
            duty = self.backend.clamp(duty, center - span, center + span)
        duty = self.backend.clamp(duty, -self.max_pwm, self.max_pwm)
        return self.vin * duty


bam.actuators.actuators.setdefault("hd1910", lambda: HD1910Actuator(Pendulum))
