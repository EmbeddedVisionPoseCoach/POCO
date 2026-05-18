import numpy as np

FEATURE_NAMES = [
    "Neck Vert Ratio", "Hand-Face Prox", "Shoulder Tilt",
    "Head Roll Ang", "Nose-Shld Hgt", "Center Offset",
    "Eye-Ear Horiz", "Fwd Head Scale", "Hand-Eye Dist",
     "Hand Visible", 
]  # 현재 피쳐 10개

MODEL_FEATURE_ORDER = [
    "eye_blink_left_mean",
    "eye_blink_left_std",
    "eye_blink_left_max",
    "eye_blink_left_min",

    "eye_blink_right_mean",
    "eye_blink_right_std",
    "eye_blink_right_max",
    "eye_blink_right_min",

    "eye_closed_score_mean",
    "eye_closed_score_std",
    "eye_closed_score_max",
    "eye_closed_score_min",

    "jaw_open_mean",
    "jaw_open_std",
    "jaw_open_max",
    "jaw_open_min",
] # 얼굴용 피쳐 16개

# ----------------------------------------------------------------
# 포즈용 피처 계산 함수
# ----------------------------------------------------------------

def calculate_features(pose_landmarks, face_landmarks=None):
    # 포즈 랜드마크가 없으면 모든 피처를 0으로 초기화하여 반환
    if not pose_landmarks:
        return [0.0] * 8

    lm = pose_landmarks[0]

    # 0. 주요 포인트 좌표 추출 (MediaPipe Pose Landmarker 기준)
    nose = lm[0]
    l_ear, r_ear = lm[7], lm[8]
    l_sh, r_sh = lm[11], lm[12]
    l_hip, r_hip = lm[23], lm[24]
    l_eye, r_eye = lm[2], lm[5]

    # 1. 어깨 너비 (기준 스케일: 모든 피처의 분모)
    sh_width = np.sqrt((l_sh.x - r_sh.x)**2 + (l_sh.y - r_sh.y)**2)
    if sh_width == 0: sh_width = 0.1 # 분모가 0이 되는 것을 방지

    # -------------------------------------------------------------------------
    # [F1] Neck Vertical Ratio (귀-어깨 수직 간격 비율)
    # 계산: (어깨 중앙 Y - 귀 중앙 Y) / 어깨너비
    # 의미: 목이 수직으로 얼마나 서 있는지를 나타냄
    # 예상: L0(정자세)에서 가장 크며, L1(거북목)에서 고개가 앞으로 숙여짐에 따라 급격히 감소함
    # -------------------------------------------------------------------------
    f1 = ((l_sh.y + r_sh.y)/2 - (l_ear.y + r_ear.y)/2) / sh_width

    # -------------------------------------------------------------------------
    # [F2] Hand-Face Proximity (손-얼굴 근접도)
    # 계산: 손 포인트들과 코 사이의 최단 거리 / 어깨너비
    # 의미: 손이 얼굴 근처로 접근하는지 측정 (턱 괴기 감지 핵심 지표)
    # 예상: L3(턱괴기)에서 0.2 이하로 급감. 손 미인식 시 1.0(안전값)을 부여하여 L3가 아님을 명시
    # -------------------------------------------------------------------------
    hand_idx = [15, 16, 17, 18, 19, 20, 21, 22]
    hand_visible = any(lm[i].visibility > 0.5 for i in hand_idx)
    if hand_visible:
        dists = [np.sqrt((lm[i].x - nose.x)**2 + (lm[i].y - nose.y)**2) for i in hand_idx]
        f2 = min(dists) / sh_width
    else:
        f2 = 1.0 # 손이 안 보이면 턱을 괴지 않은 것으로 간주 (페널티 값)

    # -------------------------------------------------------------------------
    # [F3] Shoulder Tilt Ratio (어깨 기울기 비율)
    # 계산: |왼쪽 어깨 Y - 오른쪽 어깨 Y| / 어깨너비
    # 의미: 신체의 좌우 수평 정렬(Roll) 상태를 측정
    # 예상: L0, L1에서는 0에 가깝고, L2(비대칭)에서 0.1 이상의 높은 값을 가짐
    # -------------------------------------------------------------------------
    f3 = abs(l_sh.y - r_sh.y) / sh_width

    # -------------------------------------------------------------------------
    # [F4] Head Roll Angle (고개 좌우 기울기 각도)
    # 계산: atan2(오른쪽 귀Y - 왼쪽 귀Y, 왼쪽 귀X - 오른쪽 귀X)
    # 의미: 고개가 좌(-) 또는 우(+)로 기운 각도를 도(Degree) 단위로 반환
    # 예상: L0, L1은 0도 근처. L3(턱괴기) 시 턱을 괴는 방향으로 고개가 꺾이며 큰 각도 발생
    # -------------------------------------------------------------------------
    f4 = np.degrees(np.arctan2(r_ear.y - l_ear.y, l_ear.x - r_ear.x))

    # -------------------------------------------------------------------------
    # [F5] Nose-Shoulder Height (코-어깨 높이 비율)
    # 계산: (어깨 중앙 Y - 코 Y) / 어깨너비
    # 의미: 얼굴 전체가 하단으로 얼마나 처졌는지 측정 (F1 보조)
    # 예상: L1(거북목)에서 고개를 숙일 때 F1과 함께 감소하는 경향을 보임
    # -------------------------------------------------------------------------
    f5 = ((l_sh.y + r_sh.y)/2 - nose.y) / sh_width

    # -------------------------------------------------------------------------
    # [F6] Center Offset Ratio (상체 중심 이탈 비율)
    # 계산: |코 X - 어깨 중앙 X| / 어깨너비
    # 의미: 코가 어깨의 정중앙에서 얼마나 좌우로 벗어났는지 측정 (Yaw/Lean)
    # 예상: L2(비대칭)나 턱을 괴며 몸이 쏠리는 L3에서 증가함
    # -------------------------------------------------------------------------
    f6 = abs(nose.x - (l_sh.x + r_sh.x)/2) / sh_width

   
    # -------------------------------------------------------------------------
    # [F7] Eye-Ear Horizontal Ratio (눈-귀 수평 간격 비율)
    # 계산: |눈 X - 귀 X| / 어깨너비
    # 의미: 고개가 좌우로 회전(Yaw)하는 정도를 측정
    # 예상: 정면 응시 시 일정 값을 유지하다가 고개를 옆으로 돌리면 수치가 변함
    # -------------------------------------------------------------------------
    f7 = abs(l_eye.x - l_ear.x) / sh_width

    # # -------------------------------------------------------------------------
    # # [F8] Forward Head Z-Depth (Z축 거북목 심도)
    # # 계산: ((왼쪽어깨Z + 오른쪽어깨Z)/2 - 코Z) / 어깨너비
    # # 의미: 어깨 평면 대비 코가 카메라 방향으로 돌출된 정도 측정
    # # 예상: 정면 카메라 환경에서 고개가 앞으로 빠지는 거북목(L1) 감지의 핵심 지표
    # # -------------------------------------------------------------------------
    # sh_avg_z = (l_sh.z + r_sh.z) / 2
    # f8 = (sh_avg_z - nose.z) / sh_width # 내부 계산용 변수
    # >>>>  확인 결과 크게 값이 튀어서 피쳐로 부적절하다 판단

    # -------------------------------------------------------------------------
    # [F8] Head-to-Shoulder Width Ratio (원근법 기반 거북목 깊이)
    # 계산: (양쪽 귀 사이 거리) / (양쪽 어깨 사이 거리)
    # 의미: 고개가 앞으로 나올수록 카메라에 머리가 상대적으로 크게 보이는 원근법 원리 이용
    # 예상: Z축 데이터보다 노이즈에 강하며, 거북목 시 수치가 유의미하게 상승함
    # -------------------------------------------------------------------------
    ear_width = np.sqrt((l_ear.x - r_ear.x)**2 + (l_ear.y - r_ear.y)**2)
    f8 = ear_width / sh_width 

    # -------------------------------------------------------------------------
    # [F9] Hand-Eye Distance (손-눈 최단 거리)
    # 계산: 손 포인트들과 눈 사이의 최단 거리 / 어깨너비
    # 의미: 손이 눈 주변(단순 접촉)에 있는지 확인
    # 예상: 눈 비빔 등과 턱괴기를 구분하기 위한 보조 지표
    # -------------------------------------------------------------------------
    if hand_visible:
        dists_eye = [np.sqrt((lm[i].x - l_eye.x)**2 + (lm[i].y - l_eye.y)**2) for i in hand_idx]
        f9 = min(dists_eye) / sh_width
    else:
        f9 = 1.0

    
    # -------------------------------------------------------------------------
    # [F10] Hand Visibility Flag (손 인식 여부 플래그)
    # 계산: 1.0 (인식됨) 또는 0.0 (미인식)
    # 의미: 현재 데이터 중 손 관련 피처(F2, F9, F10)의 유효성 전달
    # 예상: 모델이 손이 없을 때의 노이즈 데이터를 필터링하도록 도움
    # -------------------------------------------------------------------------
    f10 = 1.0 if hand_visible else 0.0


   

    
    return [f1, f2, f3, f4, f5, f6, f7, f8, f9, f10]

# ----------------------------------------------------------------
# 얼굴용 피처 계산 함수
# ----------------------------------------------------------------

def calculate_face_feature(blendshape_window) :
    """
    30초 동안 모은 MediaPipe face_blendshapes 값을 받아서
    모델 입력용 최종 16개 feature 리스트로 변환한다.

    Parameters
    ----------
    blendshape_window:
        30초 동안 모은 프레임별 face_blendshapes 리스트.

        권장 형태:
            [
                face_res.face_blendshapes[0],
                face_res.face_blendshapes[0],
                face_res.face_blendshapes[0],
                ...
            ]

        즉, face_res 전체를 넣는 것이 아니라
        매 프레임마다 face_res.face_blendshapes[0]만 저장해서 넣는다.

    Returns
    -------
    Optional[List[float]]

        정상 계산:
            16개 feature 리스트 반환

        계산 불가:
            None 반환

    최종 반환 순서:
        MODEL_FEATURE_ORDER와 동일하다.
    """

    # ------------------------------------------------------------
    # 1. 입력값 검사
    # ------------------------------------------------------------
    if blendshape_window is None or len(blendshape_window) == 0:
        return None

    # ------------------------------------------------------------
    # 2. 30초 동안의 프레임별 값 저장 리스트
    # ------------------------------------------------------------
    eye_blink_left_values = []
    eye_blink_right_values = []
    eye_closed_score_values = []
    jaw_open_values = []

    # ------------------------------------------------------------
    # 3. 프레임별 blendshape 처리
    # ------------------------------------------------------------
    for blendshapes in blendshape_window:

        # 얼굴이 인식되지 않은 프레임을 None으로 저장한 경우 무시
        if blendshapes is None:
            continue

        # MediaPipe blendshape 리스트를 딕셔너리로 변환
        scores = _blendshapes_to_score_dict(blendshapes)

        if len(scores) == 0:
            continue

        # --------------------------------------------------------
        # MediaPipe 원본 blendshape 이름 기준으로 값 추출
        # --------------------------------------------------------
        eye_blink_left = scores.get("eyeBlinkLeft", 0.0)
        eye_blink_right = scores.get("eyeBlinkRight", 0.0)
        jaw_open = scores.get("jawOpen", 0.0)

        # 양쪽 눈 감김 정도 평균
        eye_closed_score = (eye_blink_left + eye_blink_right) / 2.0

        # 리스트에 저장
        eye_blink_left_values.append(float(eye_blink_left))
        eye_blink_right_values.append(float(eye_blink_right))
        eye_closed_score_values.append(float(eye_closed_score))
        jaw_open_values.append(float(jaw_open))

    # ------------------------------------------------------------
    # 4. 유효 프레임이 없으면 계산 불가
    # ------------------------------------------------------------
    if len(eye_blink_left_values) == 0:
        return None

    # ------------------------------------------------------------
    # 5. 30초 단위 통계 feature 계산
    # ------------------------------------------------------------
    feature_dict = {
        "eye_blink_left_mean": _mean(eye_blink_left_values),
        "eye_blink_left_std": _std(eye_blink_left_values),
        "eye_blink_left_max": max(eye_blink_left_values),
        "eye_blink_left_min": min(eye_blink_left_values),

        "eye_blink_right_mean": _mean(eye_blink_right_values),
        "eye_blink_right_std": _std(eye_blink_right_values),
        "eye_blink_right_max": max(eye_blink_right_values),
        "eye_blink_right_min": min(eye_blink_right_values),

        "eye_closed_score_mean": _mean(eye_closed_score_values),
        "eye_closed_score_std": _std(eye_closed_score_values),
        "eye_closed_score_max": max(eye_closed_score_values),
        "eye_closed_score_min": min(eye_closed_score_values),

        "jaw_open_mean": _mean(jaw_open_values),
        "jaw_open_std": _std(jaw_open_values),
        "jaw_open_max": max(jaw_open_values),
        "jaw_open_min": min(jaw_open_values),
    }

    # ------------------------------------------------------------
    # 6. 학습 때 사용한 순서대로 리스트 생성
    # ------------------------------------------------------------
    final_features = [
        float(feature_dict[name])
        for name in MODEL_FEATURE_ORDER
    ]

    return final_features
# ----------------------------------------------------------------
# Face Feature 계산에 필요한 보조 함수들 
# ----------------------------------------------------------------

def _blendshapes_to_score_dict(blendshapes):
    """
    MediaPipe face_blendshapes 값을
    {category_name: score} 형태로 변환한다.
    """

    # 테스트용으로 dict가 바로 들어오는 경우도 허용
    if isinstance(blendshapes, dict):
        return {
            str(name): float(score)
            for name, score in blendshapes.items()
        }

    scores = {}

    for category in blendshapes:
        try:
            name = category.category_name
            score = category.score
            scores[name] = float(score)
        except AttributeError:
            continue

    return scores


def _mean(values) :
    """
    평균 계산 함수
    """

    if len(values) == 0:
        return 0.0

    return sum(values) / len(values)


def _std(values) :
    """
    표준편차 계산 함수

    pandas std()와 동일하게 sample standard deviation 방식을 사용한다.
    즉, n이 아니라 n-1로 나눈다.
    """

    n = len(values)

    if n <= 1:
        return 0.0

    mean_value = _mean(values)

    variance = sum(
        (value - mean_value) ** 2
        for value in values
    ) / (n - 1)

    return variance ** 0.5


def get_feature_names() :
    """
    최종 모델 입력 feature 이름 16개 반환
    """

    return MODEL_FEATURE_ORDER.copy()

def calculate_face_features_for_window(face_blendshapes_result):
    """
    MediaPipe Face Blendshapes 결과에서 eyeBlinkLeft, eyeBlinkRight, 
    Blink 평균, jawOpen 4가지 피처를 추출하여 리스트로 반환합니다.
    """
    # 1. 결과 데이터가 유효한지 검증 (리스트가 비어있거나 감지가 안 된 경우 예외처리)
    if not face_blendshapes_result or len(face_blendshapes_result) == 0:
        return None

    try:
        # MediaPipe는 이미지 내 감지된 얼굴 수만큼 리스트로 결과를 반환하므로, 
        # 첫 번째 얼굴([0])의 blendshapes 데이터를 가져옵니다.
        first_face_blendshapes = face_blendshapes_result[0]
        
        # 2. 접근을 용이하게 하기 위해 { '블랜드쉐이프_이름': score_value } 형태의 딕셔너리 생성
        blendshape_dict = {
            category.category_name: category.score 
            for category in first_face_blendshapes
        }
        
        # 3. 원하는 특정 피처 값 추출 (정확한 MediaPipe 공식 명칭 기준)
        # 만약 키가 없을 경우를 대비해 기본값 0.0 설정
        eye_blink_left = blendshape_dict.get('eyeBlinkLeft', 0.0)
        eye_blink_right = blendshape_dict.get('eyeBlinkRight', 0.0)
        jaw_open = blendshape_dict.get('jawOpen', 0.0)
        
        # 4. 왼쪽, 오른쪽 블링크 값의 평균 계산
        blink_average = (eye_blink_left + eye_blink_right) / 2.0
        
        # 5. 요청하신 순서대로 4개의 피처를 리스트로 조립하여 반환
        return [eye_blink_left, eye_blink_right, blink_average, jaw_open]

    except Exception as e:
        print(f"Face feature extraction error: {e}")
        return None