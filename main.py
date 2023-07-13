"""
Launch the ErgocycleS2M application.
"""
from ergocycleS2M.application.application import Application

if __name__ == "__main__":
    save_period = 0.1  # seconds
    app = Application(save_period=save_period)
    app.start()