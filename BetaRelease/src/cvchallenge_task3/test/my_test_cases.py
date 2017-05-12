import unittest

class CaseA(unittest.TestCase):

    def runTest(self):
        my_var = True

        # do some things to my_var which might change its value...

        self.assertTrue(my_var)

class CaseB(unittest.TestCase):

    def runTest(self):
        my_var = True

        # do some things to my_var which might change its value...

        self.assertTrue(my_var)


class MyTestSuite(unittest.TestSuite):

    def __init__(self):
        super(MyTestSuite, self).__init__()
        self.addTest(CaseA())
        self.addTest(CaseB())