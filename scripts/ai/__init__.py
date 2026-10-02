# -*- coding: utf-8 -*-
"""[2026_AISW] AI 구간(GPS 음영 / 회전교차로) 모듈.

rospy 에 의존하지 않는다 → 주행 노드(master_v2)와 오프라인 학습(tools/train_policy.py)이
같은 특징 추출 코드를 공유해, 학습 때와 주행 때 입력이 어긋나지 않게 한다.
"""
