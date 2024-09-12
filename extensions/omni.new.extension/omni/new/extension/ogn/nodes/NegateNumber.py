"""
This is the implementation of the OGN node defined in NegateNumber.ogn
"""

# Array or tuple values are accessed as numpy arrays so you probably need this import
import numpy


class NegateNumber:
    """
    Negates a number
    """

    @staticmethod
    def compute(db) -> bool:
        """Compute the outputs from the current input"""
        db.outputs.b = -db.inputs.a
        return True
