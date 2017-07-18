/** <module> dialog_management

@author Sascha Jongebloed 
@license BSD
*/

% defining functions
:- module(dialog_management,
    [
      assert_dialog_element/1,
      create_dialog_element/2,
      extract_guest_id/2,
      extract_query_element/2,
      assert_query_properties/2
    ]).

assert_dialog_element(JSONString) :-
    create_dialog_element(JSONString,_).

create_dialog_element(JSONString,DialogElement) :-
    atom(JSONString),
    rdf_instance_from_class(knowrob:'DialogElement', DialogElement),
    extract_guest_id(JSONString,GuestID),
    rdf_assert(DialogElement,knowrob:'guestID',GuestID),
    extract_query_element(JSONString,Query),
    rdf_assert(DialogElement,knowrob:'dialogQuery',Query).

extract_guest_id(JSONString,GuestID) :-
	atomic_list_concat([RelevantGuestString|_], ',', JSONString),
	atom_concat('{guestid:', GuestID, RelevantGuestString).

extract_query_element(JSONString,DialogQuery) :-
    rdf_instance_from_class(knowrob:'DialogQuery', DialogQuery),
	atomic_list_concat([_|[SecondHalf|_]], ',query:{', '{guestid:1,query:{type:setCake,amount:1,name:arthur}}'),
	sub_atom(SecondHalf, 0, _, 2, QueryString), % remove unneeded }}
	atomic_list_concat(QueryElements, ',', QueryString),
	assert_query_properties(QueryElements,DialogQuery).

assert_query_properties([],_).
assert_query_properties([QueryElement|Rest],DialogQuery) :-
	Ns = knowrob,
	atomic_list_concat([Property|[Value|_]],':',QueryElement),
	rdf_global_id(Ns:Property, PropertyName),
	rdf_assert(DialogQuery,PropertyName,Value),
	assert_query_properties(Rest,DialogQuery).

	