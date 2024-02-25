import cv2

cap = cv2.VideoCapture('./volleyball_match.mp4')

object_detector = cv2.createBackgroundSubtractorMOG2()

while True:
    ret, frame = cap.read()

    # extract ROI

    roi = frame[0: 540, 300: 1030]

    mask = object_detector.apply(frame)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
    for cnt in contours:
        # Calculate area
        M = cv2.moments(cnt)

        area = cv2.contourArea(cnt)

        if 260 < area < 600:
            cv2.drawContours(frame, [cnt], -1, (0, 255, 0), 2)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                # draw the contour and center of the shape on the image
                cv2.drawContours(frame, [cnt], -1, (0, 255, 0), 2)
                cv2.circle(frame, (cX, cY), 7, (255, 255, 255), -1)
                cv2.putText(frame, "center", (cX - 20, cY - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    cv2.imshow("roi", roi)
    cv2.imshow("Frame", frame)
    cv2.imshow("Mask", mask)

    if cv2.waitKey(30) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
