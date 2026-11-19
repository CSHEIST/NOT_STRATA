from PIL import Image, ImageDraw

width = 400
height = 200

img = Image.new("L", (width, height), 255)
draw = ImageDraw.Draw(img)

draw.rectangle([0, 0, width-1, height-1], outline=0, width=5)

draw.rectangle([150, 50, 250, 150], fill=0)

img.save("map.png")

print("Map generated: map.png")