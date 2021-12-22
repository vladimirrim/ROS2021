from gym_duckietown.tasks.task_solution import TaskSolution
import numpy as np
import cv2


class DontCrushDuckieTaskSolution(TaskSolution):
    def __init__(self, generated_task):
        super().__init__(generated_task)

    def solve(self):
        env = self.generated_task['env']
        # getting the initial picture
        img, _, _, _ = env.step([0, 0])

        self.move_to_duck(env)
        self.overtaking(env)
        img, reward, done, info = env.step([1, 0])
        env.render()

    def overtaking(self, env):
        self.turn_left(env)
        self.overtake(env)
        self.turn_right(env)

    def overtake(self, env):
        condition = True
        while condition:
            img, reward, done, info = env.step([1, 0])
            # img in RGB
            condition = cv2.inRange(cv2.cvtColor(img, cv2.COLOR_RGB2HSV),
                                    (20, 100, 100), (30, 255, 255)).sum() > 4e6
            env.render()

    def turn_right(self, env):
        img, _, _, _ = env.step(np.array([0, -100]))
        env.render()
        for _ in range(10):
            img, reward, done, info = env.step([1, 0])
            env.render()

        img, _, _, _ = env.step(np.array([0, 100]))
        env.render()

    def turn_left(self, env):
        img, _, _, _ = env.step(np.array([0, 100]))
        env.render()
        for _ in range(10):
            img, reward, done, info = env.step([1, 0])
            env.render()

        img, _, _, _ = env.step(np.array([0, -100]))
        env.render()

    def move_to_duck(self, env):
        condition = True
        while condition:
            img, reward, done, info = env.step([1, 0])
            # img in RGB
            condition = cv2.inRange(cv2.cvtColor(img, cv2.COLOR_RGB2HSV),
                                    (20, 100, 100), (30, 255, 255)).sum() < 5e6
            env.render()
