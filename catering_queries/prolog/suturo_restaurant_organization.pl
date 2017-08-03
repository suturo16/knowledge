/** <module> suturo_restaurant_organization

@author Sascha Jongebloed 
@license BSD
*/

% defining functions
:- module(suturo_restaurant_organization,
    [
      get_customer_infos/3,
      get_open_orders_of/4,
      get_open_orders_with_customer_infos/6,
      get_free_table/1,
      set_delivered_amount/2,
      increase_delivered_amount/2,
      increase_delivered_amount/1
    ]).

:- rdf_meta get_customer_infos(?,?,r),
      get_open_orders_of(?,?,?,?),
      get_open_orders_with_customer_infos(?,?,r,?,?,?),
      get_free_table(?),
      set_delivered_amount(?,?),
      increase_delivered_amount(?,?),
      increase_delivered_amount(?).

%getCustomerInfos(+CustomerID,-Name,-Place)
get_customer_infos(CustomerID,Name,Place) :-
      get_open_orders_with_customer_infos(CustomerID,Name,Place,_,_,_) .


%getOpenOrdersOf(+customerID,-Item,-Amount)
get_open_orders_of(CustomerID,Item,TotalAmount,DeliveredAmount) :-
      get_open_orders_with_customer_infos(CustomerID,_,_,Item,TotalAmount,DeliveredAmount).


%getOpenOrdersWithCustomerInfos(+CustomerID,-Name,-Place,-Good,-Amount)
get_open_orders_with_customer_infos(CustomerID,Name,Place,Item,TotalAmount,DeliveredAmount) :-
    rdf_has(Obj,knowrob:'guestId',literal(type(xsd:string,CustomerID))), %# rdf_has because holds gives redundant results
    rdf_has(Obj,rdf:type,knowrob:'Customer'), %# rdf_has because holds gives redundant results
    holds(Obj,knowrob:'guestName',literal(type(xsd:string,Name))),
    ignore(holds(Obj,knowrob:'guestName',literal(type(xsd:string,Name)))),
    holds(Obj,knowrob:'visit',Visit),
    ignore(holds(Visit,knowrob:'locatedAt',Place)),
    holds(Visit,knowrob:'hasOrder',Order),
    holds(Order,knowrob:'itemName',literal(type(xsd:string,Item))),
    holds(Order,knowrob:'orderedAmount',literal(type(xsd:integer,TotalAmountAtom))),
    (atom(TotalAmountAtom) -> atom_number(TotalAmountAtom,TotalAmount) ; TotalAmount = TotalAmountAtom),
    holds(Order,knowrob:'deliveredAmount',literal(type(xsd:integer,DeliveredAmountAtom))), %# This parameter isn#t always needed
    (atom(DeliveredAmountAtom) -> atom_number(DeliveredAmountAtom,DeliveredAmount) ; DeliveredAmount = DeliveredAmountAtom).

%getFreeTables(-NameOfFreeTable)
get_free_table(NameOfFreeTable) :-
      owl_has(Table,rdf:type,knowrob:'RestaurantTable'),
      owl_has(Table,knowrob:'tableId',literal(type(xsd:string,NameOfFreeTable))),
      \+ (holds(_,knowrob:'locatedAt',Table)).

% set_delivered_amount(+CustomerID,+Amount)
set_delivered_amount(CustomerID,Amount) :-
    (holds(Obj,knowrob:'guestId',literal(type(xsd:string,CustomerID))),
    holds(Obj,rdf:type,knowrob:'Customer'),!),
    holds(Obj,knowrob:'visit',Visit),
    holds(Visit,knowrob:'hasOrder',Order),
    %# Replace the deliveredAmount value by the new value
    current_time(Now),
    forall((holds(Order,knowrob:'deliveredAmount',O)), 
    assert_temporal_part_end(Order, knowrob:'deliveredAmount', O, Now)),
    assert_temporal_part(Order, knowrob:'deliveredAmount', Amount).

% increase_delivered_amount(+CustomerID)
increase_delivered_amount(CustomerID,Amount) :-
    (holds(Obj,knowrob:'guestId',literal(type(xsd:string,CustomerID))),
    holds(Obj,rdf:type,knowrob:'Customer'),!),
    holds(Obj,knowrob:'visit',Visit),
    holds(Visit,knowrob:'hasOrder',Order),
    holds(Order,knowrob:'deliveredAmount',literal(type(xsd:integer,DeliveredAmount))),
    (atom(DeliveredAmount) -> atom_number(DeliveredAmount,DAAsNumber);DAAsNumber = DeliveredAmount),
    (atom(Amount) -> atom_number(Amount,AmountAsNumber);AmountAsNumber = Amount),
    DeliveredAmountNew is DAAsNumber + AmountAsNumber,
    set_delivered_amount(CustomerID,DeliveredAmountNew).

increase_delivered_amount(CustomerID) :-
    increase_delivered_amount(CustomerID,1).


