(define (problem problem0)
(:domain office)
(:objects
user - user
r - robot
coffee - content
sandwich - content
officeOfAnna - office
Anna - person
officeOfEva - office
Eva - person
officeOfLena - office
officeOfMalin - office
officeOfKarin - office
officeOfEmma - office
officeOfJohanna - office
Lena - person
Malin - person
Karin - person
Emma - person
Johanna - person
Hanna - person
Helena - person
Åsa - person
officeOfHanna - office
officeOfHelena - office
officeOfÅsa - office
officeOfElin - office
Elin - person
Annika - person
Johan - person
Anders - person
officeOfAnnika - office
officeOfJohan - office
officeOfAnders - office
officeOfFredrik - office
officeOfMagnus - office
Fredrik - person
Magnus - person
Andreas - person
Mikael - person
officeOfAndreas - office
officeOfMikael - office
officeOfJonas - office
officeOfErik - office
Jonas - person
Erik - person
Stefan - person
Lars - person
officeOfStefan - office
officeOfLars - office
officeOfPer - office
officeOfHenrik - office
Per - person
Henrik - person
vendingMachine0 - vending
vendingMachine1 - vending
vendingMachine2 - vending
vendingMachine3 - vending
)
(:init
(robotEmpty r)
(robotAt r user)
(vendingHas vendingMachine0 sandwich)
(vendingHas vendingMachine1 coffee)
(vendingHas vendingMachine2 sandwich)
(vendingHas vendingMachine3 coffee)
(personNeed Annika coffee)
)
(:goal (and
(personHas Annika coffee)	))
)
