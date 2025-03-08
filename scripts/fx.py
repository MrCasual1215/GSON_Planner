#!/usr/bin/env python

def hex_to_rgb(hex_color):
    """
    Convert a hex color code to RGB values in the range of 0 to 1.
    :param hex_color: str, hex color code (e.g., '393D3F')
    :return: tuple, RGB values in range of 0 to 1 (e.g., (0.2235, 0.2392, 0.2471))
    """
    # Ensure hex_color is in proper format
    hex_color = hex_color.lstrip('#')
    
    # Convert hex to RGB (0-255)
    rgb_255 = tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))
    
    # Convert RGB (0-255) to RGB (0-1)
    rgb_normalized = tuple(value / 255 for value in rgb_255)
    
    return rgb_normalized

def main():
    # Ask user for hex input
    hex_color = input("Please enter a hex color code (without #): ")
    
    try:
        # Convert the input hex color to RGB
        rgb = hex_to_rgb(hex_color)
        print(f"RGB values in range (0-1) for {hex_color}: {rgb}")
        print(f"RGB values in range (0-255) for {hex_color}: {rgb[0]*255, rgb[1]*255, rgb[2]*255}")
        
    except ValueError:
        print("Invalid hex color code. Please provide a valid 6-character hex code.")

if __name__ == "__main__":
    main()
