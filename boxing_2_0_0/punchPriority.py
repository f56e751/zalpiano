from punch import Punch
class PunchPriority():
    def __init__(self, Left: Punch, Right: Punch):
        self.Left = Left
        self.Right = Right



    def getPriorityPunch(self):
        # TODO 펀치 종류, 속도, 등에 따른 우선순위 정하기
        return self.Right