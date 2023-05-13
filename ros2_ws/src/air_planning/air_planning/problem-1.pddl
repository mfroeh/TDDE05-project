(define (problem problem0)
(:domain office)
(:objects
user - user
r - robot
coffee - content
sandwich - content
officeOfanna - office
anna - person
officeOfeva - office
eva - person
officeOflena - office
officeOfmalin - office
officeOfkarin - office
officeOfemma - office
officeOfjohanna - office
lena - person
malin - person
karin - person
emma - person
johanna - person
hanna - person
helena - person
officeOfhanna - office
officeOfhelena - office
officeOfelin - office
elin - person
annika - person
johan - person
anders - person
officeOfannika - office
officeOfjohan - office
officeOfanders - office
officeOffredrik - office
officeOfmagnus - office
fredrik - person
magnus - person
andreas - person
mikael - person
officeOfandreas - office
officeOfmikael - office
officeOfjonas - office
officeOferik - office
jonas - person
erik - person
stefan - person
lars - person
officeOfstefan - office
officeOflars - office
officeOfper - office
officeOfhenrik - office
per - person
henrik - person
vendingMachineid98 - vending
vendingMachineid96 - vending
vendingMachineid97 - vending
)
(:init
(robotEmpty r)
(robotAt r user)
(vendingHas vendingMachineid98 sandwich)
(vendingHas vendingMachineid96 coffee)
(vendingHas vendingMachineid96 sandwich)
(vendingHas vendingMachineid97 coffee)
(personNeed erik coffee)
)
(:goal (and
(personHas erik coffee)
	)
)
)
