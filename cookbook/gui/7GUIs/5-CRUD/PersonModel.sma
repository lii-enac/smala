use core

_define_
PersonModel (string _name, string _surname)
{
    String name (_name)         // Forename
    String surname (_surname)   // Surname
    
    // For log purpose (can be made directly in the view)
    String fullname (_surname + ", " + _name)
    surname + ", " + name => fullname

    // TextPrinter tp
    // // "Model of Person: (fore)name = " + name + " - surname " + surname =:> tp.input
    // "Model of Person: '" + name + "' + '" + surname + "' --> " + fullname =:> tp.input
}