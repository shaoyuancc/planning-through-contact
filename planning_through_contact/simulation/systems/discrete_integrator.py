from pydrake.all import LeafSystem

import logging

logger = logging.getLogger(__name__)

class DiscreteIntegrator(LeafSystem):
    def __init__(self, size):
        LeafSystem.__init__(self)
        self._size = size
        # self._period = period
        self._rate_port = self.DeclareVectorInputPort("rate", size)
        self._output_port = self.DeclareVectorOutputPort("integral_value", size, self.CalcOutput)
        # self._integral_value = self.DeclareDiscreteState([0.0]*size)
        self._last_time = self.DeclareDiscreteState([0.0])
        self._integral_value = [0.0]*size
    
    def CalcOutput(self, context, output):
        rate = self._rate_port.Eval(context)
        # integral_value = self._integral_value.get_value()
        last_time_state = context.get_discrete_state(self._last_time)
        time_step = context.get_time() - last_time_state.get_value()[0]

        integral_value = self._integral_value
        # print(f"rate: {rate}, time_step: {time_step}, integral_value: {integral_value}")
        integral_value += rate * time_step
        output.set_value(integral_value)
        last_time_state.set_value([context.get_time()])
        self._integral_value = integral_value
    
    def set_integral_value(self, integral_value):
        # print(f"set_integral_value: {integral_value}")
        self._integral_value = integral_value
        # context.get_mutable_discrete_state().get_mutable_vector().SetFromVector(integral_value)