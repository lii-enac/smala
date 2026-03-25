use core

_define_
PersonModel (string _name, string _surname)
{
    String name (_name)
    String surname (_surname)
    String fullname (_surname + ", " + _name)   // Can be made in the view

    TextPrinter tp
    "Model of Person: " + name + " " + surname + " --> " + fullname =:> tp.input
}