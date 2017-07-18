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
      increase_delivered_amount/1
    ]).

%getCustomerInfos(+CustomerID,-Name,-Place)
get_customer_infos(CustomerID,Name,Place) :-
      get_open_orders_with_customer_infos(CustomerID,Name,Place,_,_) .


%getOpenOrdersOf(+customerID,-Item,-Amount)
get_open_orders_of(CustomerID,Item,TotalAmount,Delivered) :-
      get_open_orders_with_customer_infos(CustomerID,_,_,Item,TotalAmount,DeliveredAmount).


%getOpenOrdersWithCustomerInfos(+CustomerID,-Name,-Place,-Good,-Amount)
get_open_orders_with_customer_infos(CustomerID,Name,Place,Item,TotalAmount,DeliveredAmount) :-
      Name='michael',
      Place='table1',
      Item='cake',
      TotalAmount=1,
      DeliveredAmount=0.

%getFreeTables(-NameOfFreeTable)
get_free_table(NameOfFreeTable) :-
      owl_has(Table,rdf:type,knowrob:'RestaurantTable'),
      \+ (owl_has(_,knowrob:'locatedAt',Table)).

% set_delivered_amount(+CustomerID,+Amount)
set_delivered_amount(CustomerID,Amount) :-
    true.

% increase_delivered_amount(+CustomerID)
increase_delivered_amount(CustomerID) :-
  true.
