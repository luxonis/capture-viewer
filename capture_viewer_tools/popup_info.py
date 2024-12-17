from tkinter import messagebox

def show_popup(info_text):
    if info_text[0] == '{':
        text = '{'  # assuming the string is a json for better readibility
        i = 1
    else:
        text = ''
        i = 0
    indent = 0
    while i < len(info_text):
        # Handle opening braces and nested objects
        if info_text[i] == '{' or info_text[i] == '[':
            text += '\n' + '\t' * indent + info_text[i] + '\n'
            indent += 1
            text += '\t' * indent
            i += 1
        # Handle closing braces and nested objects
        elif info_text[i] == '}' or info_text[i] == ']':
            indent -= 1
            text += '\n' + '\t' * indent + info_text[i]
            i += 1
        # Handle commas
        elif info_text[i] == ',':
            text += ',\n' + '\t' * indent
            i += 1
        # Handle regular characters
        else:
            text += info_text[i]
            i += 1
    # Show the formatted JSON in the popup window
    messagebox.showinfo("Point Cloud generated with:", text)

if __name__ == "__main__":
    # Example multi-level JSON with "lorem ipsum" text
    lorem_json = '''
    {
        "title": "Lorem Ipsum Example",
        "author": {
            "name": "John Doe",
            "bio": "Lorem ipsum dolor sit amet, consectetur adipiscing elit. Fusce in urna id nulla cursus facilisis.",
            "address": {
                "street": "123 Main St",
                "city": "Metropolis",
                "zipcode": "12345"
            },
            "affiliations": [
                {
                    "company": "Lorem Industries",
                    "role": "Senior Developer",
                    "start_date": "2021-01-01",
                    "end_date": null
                },
                {
                    "company": "Ipsum Corp",
                    "role": "Software Engineer",
                    "start_date": "2019-06-15",
                    "end_date": "2020-12-30"
                }
            ]
        },
        "content": {
            "introduction": "Lorem ipsum dolor sit amet, consectetur adipiscing elit. Sed euismod urna eu feugiat volutpat.",
            "chapters": [
                {
                    "chapter_number": 1,
                    "title": "The Beginning",
                    "text": "Lorem ipsum dolor sit amet, consectetur adipiscing elit. Vivamus scelerisque tristique magna."
                },
                {
                    "chapter_number": 2,
                    "title": "The Journey Continues",
                    "text": "Phasellus at volutpat mauris. Integer viverra purus eget ligula tincidunt, nec fermentum justo feugiat."
                },
                {
                    "chapter_number": 3,
                    "title": "The End",
                    "text": "Cras convallis libero a velit placerat, a iaculis libero cursus. Morbi ut magna id ante pharetra mollis."
                }
            ]
        },
        "conclusion": "Lorem ipsum dolor sit amet, consectetur adipiscing elit. Nunc auctor dui ac eros malesuada, sit amet laoreet purus pharetra."
    }
    '''

    show_popup(lorem_json)