from pydrake.systems.framework import LeafSystem

class MimicFingerSystem(LeafSystem):
    def __init__(self, plant, left_finger_joint, right_finger_joint):
        LeafSystem.__init__(self)
        self.plant = plant
        self.left_finger_joint = left_finger_joint
        self.right_finger_joint = right_finger_joint

        # Declare input port for the plant's state
        self.plant_state_input_port = self.DeclareVectorInputPort(
            "plant_state", self.plant.num_multibody_states())

        # Declare output port for the plant's state (with mimic behavior)
        self.DeclareVectorOutputPort(
            "plant_state_with_mimic", self.plant.num_multibody_states(),
            self.CalcOutput)

    def CalcOutput(self, context, output):
        # Get the current state of the plant
        plant_state = self.plant_state_input_port.Eval(context)
        self.plant.SetPositionsAndVelocities(context, plant_state)

        # Get the current position of the left finger
        left_finger_position = self.left_finger_joint.translation(context)

        # Set the right finger's position to the negative of the left finger's position
        self.right_finger_joint.set_translation(context, -left_finger_position)

        # Output the updated state
        output.SetFromVector(self.plant.GetPositionsAndVelocities(context))