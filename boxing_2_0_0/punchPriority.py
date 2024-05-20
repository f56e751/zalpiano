import sys
import os

# boxing_2_0_0 디렉토리의 부모 디렉토리를 sys.path에 추가
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from boxing_2_0_0 import Punch
class PunchPriority():
    def __init__(self, Left: Punch, Right: Punch):
        self.Left = Left
        self.Right = Right



    def getPriorityPunch(self):
        # TODO 펀치 종류, 속도, 등에 따른 우선순위 정하기
        return self.Right