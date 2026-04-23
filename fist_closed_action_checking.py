import cv2
import mediapipe as mp
import time


class GestureController:

    def __init__(self):

        self.cap = cv2.VideoCapture(0)

        self.mp_hands = mp.solutions.hands
        self.mp_draw = mp.solutions.drawing_utils

        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.6
        )

    # ------------------------
    # Finger counting
    # ------------------------
    def count_fingers(self, hand_landmarks, hand_label):

        lm = hand_landmarks.landmark
        tips = [4,8,12,16,20]

        count = 0

        if hand_label == "Right":
            if lm[tips[0]].x < lm[tips[0]-1].x:
                count += 1
        else:
            if lm[tips[0]].x > lm[tips[0]-1].x:
                count += 1

        for t in tips[1:]:
            if lm[t].y < lm[t-2].y:
                count += 1

        return count


    # ------------------------
    # Left fist/open
    # ------------------------
    def left_state(self, hand_landmarks):

        lm = hand_landmarks.landmark

        fingers = 0
        for t in [8,12,16,20]:
            if lm[t].y < lm[t-2].y:
                fingers += 1

        # 0 = closed, 1 = open
        if fingers == 0:
            return 0
        else:
            return 1


    # ------------------------
    # Continuous camera feed
    # ------------------------
    def update_camera(self):
        """
        Non-blocking single camera update.
        Call repeatedly inside your robot loop.
        """
    
        ok, frame = self.cap.read()
        if not ok:
            return
    
        frame = cv2.flip(frame,1)
    
        cv2.putText(
            frame,
            "Camera Active",
            (20,40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0,255,0),
            2
        )
    
        cv2.imshow("Gesture Interface", frame)
    
        cv2.waitKey(1)


    # ------------------------
    # Capture gesture on demand
    # ------------------------
    def get_gesture(self):

        start = time.time()

        left_result = None
        right_result = None

        while time.time()-start < 3:

            ok, frame = self.cap.read()
            if not ok:
                break

            frame = cv2.flip(frame,1)

            rgb = cv2.cvtColor(
                frame,
                cv2.COLOR_BGR2RGB
            )

            results = self.hands.process(rgb)

            cv2.putText(
                frame,
                "Gesture Required",
                (50,70),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0,0,255),
                3
            )

            if results.multi_hand_landmarks:

                for hand_landmarks, handedness in zip(
                    results.multi_hand_landmarks,
                    results.multi_handedness
                ):

                    label = handedness.classification[0].label

                    self.mp_draw.draw_landmarks(
                        frame,
                        hand_landmarks,
                        self.mp_hands.HAND_CONNECTIONS
                    )

                    if label=="Left":
                        left_result = self.left_state(
                            hand_landmarks
                        )

                    if label=="Right":
                        right_result = self.count_fingers(
                            hand_landmarks,
                            label
                        )

            cv2.imshow(
                "Gesture Interface",
                frame
            )

            if cv2.waitKey(1)==27:
                break


        # ------------------------
        # Hold response for 1 sec
        # ------------------------

        hold_start=time.time()

        while time.time()-hold_start < 3:

            ok, frame = self.cap.read()
            if not ok:
                break
            frame = cv2.flip(frame,1)

            cv2.putText(
                frame,
                "Response Gathered",
                (50,70),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0,255,255),
                3
            )

            cv2.putText(
                frame,
                f"Left={left_result}, Right={right_result}",
                (50,130),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255,0,0),
                2
            )

            cv2.imshow(
                "Gesture Interface",
                frame
            )

            cv2.waitKey(1)

        return left_result,right_result


    def close(self):
        self.cap.release()
        cv2.destroyAllWindows()