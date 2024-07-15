import tkinter as tk
from tkinter import scrolledtext, filedialog, messagebox

# Define the words to be marked and their corresponding colors
marked_words = {
    'Python': 'red',
    'language': 'blue',
    'code': 'green'
}

def highlight_text(event=None):
    # Remove previous highlights
    for tag in marked_words.keys():
        text_editor.tag_remove(tag, "1.0", tk.END)
    
    # Get the entire text
    text = text_editor.get("1.0", tk.END)
    
    # Highlight each marked word
    for word, color in marked_words.items():
        start = "1.0"
        while True:
            pos = text_editor.search(word, start, stopindex=tk.END)
            if not pos:
                break
            end = f"{pos}+{len(word)}c"
            text_editor.tag_add(word, pos, end)
            text_editor.tag_config(word, foreground=color)
            start = end

def save_file():
    file_path = filedialog.asksaveasfilename(defaultextension=".txt",
                                             filetypes=[("Text files", "*.txt"),
                                                        ("All files", "*.*")])
    if file_path:
        try:
            with open(file_path, 'w') as file:
                file.write(text_editor.get("1.0", tk.END))
            messagebox.showinfo("Success", "File saved successfully")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save file: {e}")

def load_file():
    file_path = filedialog.askopenfilename(filetypes=[("Text files", "*.txt"),
                                                      ("All files", "*.*")])
    if file_path:
        try:
            with open(file_path, 'r') as file:
                text_editor.delete("1.0", tk.END)
                text_editor.insert(tk.END, file.read())
            highlight_text()
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load file: {e}")

def validate_file():
    text = text_editor.get("1.0", tk.END).strip()
    # Here, add your validation logic
    if text:  # Simple validation: check if the text is not empty
        validation_label.config(text="✔", fg="green")
    else:
        validation_label.config(text="✖", fg="red")

# Create the main window
root = tk.Tk()
root.title("Text Editor with Highlighted Words")

# Create a menu bar
menu_bar = tk.Menu(root)
root.config(menu=menu_bar)

# Add a "File" menu with "Load" and "Save" options
file_menu = tk.Menu(menu_bar, tearoff=0)
file_menu.add_command(label="Load", command=load_file)
file_menu.add_command(label="Save", command=save_file)
menu_bar.add_cascade(label="File", menu=file_menu)

# Add a "Validate" button to validate the content
validate_button = tk.Button(root, text="Validate", command=validate_file)
validate_button.pack(side=tk.TOP, padx=10, pady=5)

# Add a label for validation status (green check mark or red cross)
validation_label = tk.Label(root, text="", font=("Arial", 24))
validation_label.pack(side=tk.TOP, anchor="ne", padx=10, pady=5)

# Create a ScrolledText widget
text_editor = scrolledtext.ScrolledText(root, wrap=tk.WORD, width=60, height=20)
text_editor.pack(padx=10, pady=10, fill=tk.BOTH, expand=True)

# Bind the text change event to the highlight_text function
text_editor.bind("<KeyRelease>", highlight_text)

# Run the main loop
root.mainloop()

