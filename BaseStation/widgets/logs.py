import tkinter as tk
from tkinter import scrolledtext


class Logs:
    def __init__(self, root):   
        self.log_text = scrolledtext.ScrolledText(root, wrap=tk.WORD, height=8, width=80)
        self.log_text.grid(column=0, row=0, columnspan=1, rowspan=1, sticky="nsew")

        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)

    def update_log(self, message):
        self.log_text.insert(tk.END, message)
        self.log_text.yview(tk.END)
