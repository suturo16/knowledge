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
      assert_query_properties/2,
      create_query_of_type/3,
      extract_only_type/3,
      extract_only_type_help/4,
      run_the_rules/0
    ]).

assert_dialog_element(JSONString) :-
    create_dialog_element(JSONString,_),
    run_the_rules.

run_the_rules :-
    ignore((
      swrl:rdf_swrl_name(Descr,'SetCakeWithCustomer'),
      rdf_has(Descr, knowrob:swrlActionVariable, VarLiteral),
      strip_literal_type(VarLiteral, Var),
      rdf_swrl_project(Descr, [var(Var,Act)]))),
    ignore((
      swrl:rdf_swrl_name(Descr,'SetLocation'),
      rdf_has(Descr, knowrob:swrlActionVariable, VarLiteral),
      strip_literal_type(VarLiteral, Var),
      rdf_swrl_project(Descr, [var(Var,Act)]))).


create_dialog_element(JSONString,DialogElement) :-
    atom(JSONString),
    rdf_instance_from_class(knowrob:'DialogElement', DialogElement),
    rdf_assert(DialogElement,knowrob:'checked',literal(type(xsd:boolean,false))),
    extract_guest_id(JSONString,GuestID),
    rdf_assert(DialogElement,knowrob:'guestId',literal(type(xsd:string,GuestID))),
    extract_query_element(JSONString,Query),
    rdf_assert(DialogElement,knowrob:'dialogQuery',Query).

extract_guest_id(JSONString,GuestID) :-
	atomic_list_concat([RelevantGuestString|_], ',', JSONString),
	atom_concat('{guestId:', GuestID, RelevantGuestString).

extract_query_element(JSONString,QueryInstance) :-
	atomic_list_concat([_|[SecondHalf|_]], ',query:{', JSONString),
	sub_atom(SecondHalf, 0, _, 2, QueryString), % remove unneeded '}}'
	atomic_list_concat(QueryElements, ',', QueryString),
  create_query_of_type(QueryElements,CleanedQueryElements,QueryInstance),
	assert_query_properties(CleanedQueryElements,QueryInstance).

create_query_of_type(QueryElements,CleanedQueryElements,QueryInstance) :-
    extract_only_type(QueryElements,Type,CleanedQueryElements),
    get_class_name(Type,ClassName),
    atom_concat('http://knowrob.org/kb/knowrob.owl#', ClassName, FullClassName),
    rdf_instance_from_class(FullClassName,QueryInstance).

extract_only_type(QueryElements,Type,CleanedQueryElements) :-
  extract_only_type_help(QueryElements,[],Type,CleanedQueryElements).

extract_only_type_help([],TillNow,Type,CleanedQueryElements) :-
  write('Query has no type'),
  Type = 'DialogQuery',
  CleanedQueryElements = TillNow.

extract_only_type_help([QueryElement|Rest],TillNow,Type,CleanedQueryElements) :-
  atomic_list_concat([Property|[Value|_]],':',QueryElement),
  (Property = type ->
      (Type = Value,
        append(TillNow,Rest,CleanedQueryElements));
      (append([QueryElement],TillNow,TillNowNew),
        extract_only_type_help(Rest,TillNowNew,Type,CleanedQueryElements))
    ).

assert_query_properties([],_).
assert_query_properties([QueryElement|Rest],DialogQuery) :-
	Ns = knowrob,
	atomic_list_concat([Property|[Value|_]],':',QueryElement),
	rdf_global_id(Ns:Property, PropertyName),
	rdf_assert_with_literal(DialogQuery,PropertyName,Value),
	assert_query_properties(Rest,DialogQuery).

rdf_assert_with_literal(S,P,Value) :-
  rdf_has(P, rdf:type, owl:'DatatypeProperty'),
  (  rdf_phas(P, rdfs:range, Range)
  -> rdf_assert(S, P, literal(type(Range,Value)))
  ;  rdf_assert(S, P, literal(Value))
  ), !.

%% get_class_name(+Type, -ClassName)
% MSp
% converts first letter of Type into capital letter
get_class_name(Type, ClassName) :-
    not((var(Type), var(ClassName))),
    sub_atom(Type,0,1,_,C), char_code(C,I), 96<I, I<123
    -> J is I-32, char_code(D,J), sub_atom(Type,1,_,0,Sub), atom_concat(D,Sub,ClassName)
    ; ClassName = Type.

	