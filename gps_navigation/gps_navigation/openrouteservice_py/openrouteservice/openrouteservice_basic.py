import openrouteservice

coords = ((8.34234,48.23424),(8.34423,48.26424))

client = openrouteservice.Client(key='5b3ce3597851110001cf6248c095f2a04f244c2bb56e451d005b8712')
routes = client.directions(coords)

print(routes)