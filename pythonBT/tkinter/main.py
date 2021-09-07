import tkinter as tk

# Instance root window
window = tk.Tk()

# Adding a widget
greeting = tk.Label(text="Python rocks!", bg = "black", fg="white",width=20,height=10)
button = tk.Button(text = "Click me!", width=5,height=5,bg="yellow",fg="red")
entry = tk.Entry(fg="yellow", bg="blue", width = 20)
entry.insert(0,"python")
text = tk.Text(fg="white",bg="red",width=20, height=20)

# Placing Widget into the form
greeting.pack()
entry.pack()
text.pack()
button.pack()


# Run tkinter event loop
window.mainloop()

