(define (problem problem2)
(:domain office)
(:objects
user - user
r - robot
coffee - content
sandwich - content
officeOfÅsa - office
Åsa - person
officeOfHelena - office
Helena - person
officeOfJohan - office
Johan - person
Elin - person
officeOfAnna - office
Anna - person
officeOfEva - office
Eva - person
Lena - person
Malin - person
Karin - person
Emma - person
officeOfLena - office
officeOfMalin - office
officeOfKarin - office
officeOfEmma - office
officeOfJohanna - office
officeOfHanna - office
officeOfHelena - office
officeOfÅsa - office
Johanna - person
Hanna - person
Helena - person
Åsa - person
officeOfElin - office
officeOfAnnika - office
officeOfJohan - office
officeOfAnders - office
Elin - person
Annika - person
Johan - person
Anders - person
officeOfFredrik - office
officeOfMagnus - office
officeOfAndreas - office
officeOfMikael - office
Fredrik - person
Magnus - person
Andreas - person
Mikael - person
Jonas - person
officeOfJonas - office
officeOfErik - office
officeOfStefan - office
officeOfLars - office
Erik - person
Stefan - person
Lars - person
Per - person
officeOfPer - office
officeOfHenrik - office
vendingMachine0 - vending
vendingMachine1 - vending
vendingMachine2 - vending
Henrik - person
vendingMachine3 - vending
)
(:init
(robotEmpty r)
(robotAt r user)
(vendingHas vendingMachine0 coffee)
(vendingHas vendingMachine1 sandwich)
(vendingHas vendingMachine2 coffee)
(vendingHas vendingMachine3 sandwich)
)
(:goal (and
(visited r officeOfErik)	))
)
