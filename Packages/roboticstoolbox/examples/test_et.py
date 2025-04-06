
from collections import UserList, namedtuple


class ET(UserList):
    def __init__(self, axis_func=None, axis=None, eta=None):
        super().__init__()  # init UserList superclass

        if axis_func is None and axis is None and eta is None:
            # ET()
            self.data = []
        else:
            # ET(axis_func, axis, value)
            et = namedtuple('ET', 'axis_func axis eta')  # stash the parameters in a named tuple, like a struct
            self.data = [et(axis_func, axis, eta)]  # put it into a list, the only element

    # create some properties to extract the attributes, 
    #  assuming the length is one
    @property
    def eta(self):
        return self.data[0].eta

    @property
    def axis(self):
        return self.data[0].axis

    @property
    def axis_func(self):
        return self.data[0].axis_func

    # display it, it may have multiple values, ie. len(self.data) > 1
    def __repr__(self):
        s = []
        for et in self:  # for et in the object, display it, data comes from properties which come from the named tuple
            s.append(f"{et.axis}({et.eta})")
        return " * ".join(s)

    # redefine * operator to concatenate the internal lists
    def __mul__(self, rest):
        prod = ET()
        prod.data = self.data + rest.data
        return prod

    # redefine so that indexing returns an ET type
    def __getitem__(self, i):
        item = ET()
        data = self.data[i]  # can be [2] or slice, eg. [3:5]
        # ensure that data is always a list
        if isinstance(data, list):
            item.data = data
        else:
            item.data = [data]
        return item

    # these need to be fleshed out ala ET.py, just cut and paste
    @classmethod
    def Rx(cls, eta=None):
        return cls(None, 'Rx', eta)

    @classmethod
    def Tx(cls, eta=None):
        return cls(None, 'Tx', eta)

if __name__ == "__main__":
    e = ET.Rx(0.2) * ET.Tx(0.3) * ET.Tx(0.4) * ET.Rx(0.5)
    print(len(e))
    print(e)
    z = e[2]
    print(e[0:2])

    f = e * e
    print(f)