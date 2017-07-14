/** <module> dialog_management

@author Sascha Jongebloed 
@license BSD
*/

% defining functions
:- module(dialog_management,
    [
      assert_dialog_element/1     
    ]).

assert_dialog_element(JSONString) :-
    write(JSONString),
    true.