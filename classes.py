class Link:
    def __init__(self, name, length):
        self.name = name
        self.length = length

    def __repr__(self):
        return f"Link(name={self.name}, length={self.length})"


class Robot:
    def __init__(self, name):
        self.name = name
        self.links = []

    def add_link(self, link):
        self.links.append(link)

    def __repr__(self):
        return f"Robot(name={self.name}, links={self.links})"