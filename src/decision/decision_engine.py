class DecisionEngine:
    def __init__(self):
        pass
        
    def decide(self, risk_level):
        """
        Determines the vehicle action based on risk level.
        Possible actions: CRUISE, SLOW_DOWN, STOP, REPLAN, EMERGENCY_BRAKE
        """
        if risk_level == 'CRITICAL':
            return 'EMERGENCY_BRAKE'
        elif risk_level == 'HIGH':
            return 'REPLAN'
        elif risk_level == 'MEDIUM':
            return 'SLOW_DOWN'
        else: # LOW
            return 'CRUISE'
