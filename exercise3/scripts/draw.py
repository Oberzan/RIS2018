import cv2


def draw_with_goals(img, goals):
    goal_img = img.copy()
    for p in goals:
        cv2.circle(goal_img, (p.x, p.y), 2, (0, 255, 0), -1)
    cv2.imshow('image', goal_img)
