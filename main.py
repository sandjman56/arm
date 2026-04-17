#!/usr/bin/env python3
# main.py -- Entry point for Continuum Arm Controller
import tkinter as tk
from app import ArmUI

if __name__ == "__main__":
    root = tk.Tk()
    app = ArmUI(root)
    root.mainloop()
