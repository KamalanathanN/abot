#!/usr/bin/ruby

require 'rmagick'
require 'json'
require 'minitest/autorun'
require './boustrophedon_decomposition'

class MapTests < Minitest::Test
	def setup()
		BoustrophedonDecomposition.logger.level = Logger::WARN
		BoustrophedonDecomposition.logger.formatter = proc do |severity, datetime, progname, msg|
			"#{msg}\n"
		end
	end

	def img2array(filename)
		img = Magick::Image.read(filename)[0]
		map = []
		img.columns.times.map do |x|
			pixels = img.get_pixels(x, 0, 1, img.rows) # (x, y, columns, rows)
			map[x] = pixels.map { |pixel| pixel.red>>8 } # from 16 to 8
		end
		return map
	end

	def visualize(img, cells)
		puts "--"
		puts "Cells: #{cells.length}"
		img = Magick::Image.read('test/images/map5.png')[0]
		cells.each do |cell|
			cell.points.each do |point|
				px = Magick::Pixel.new(0xffff,0,0)
				img.store_pixels(point[0], point[1], 1, 1, [px])
			end
		end
		img.display
	end

	def test_map1
		map = img2array('test/images/map.png')
		cells = BoustrophedonDecomposition.decompose(map)
		polygons = cells.map { |cell| cell.points }
		assert_equal 8, polygons.length
		assert_includes polygons, [[ 0, 74], [14, 74], [14,  0], [ 0,  0]]
		assert_includes polygons, [[16, 58], [63, 58], [63, 17], [16, 17]]
		assert_includes polygons, [[64, 26], [72, 26], [72, 17], [64, 17]]
		assert_includes polygons, [[64, 58], [72, 58], [72, 32], [64, 32]]
		assert_includes polygons, [[73, 58], [87, 58], [87, 17], [73, 17]]
		assert_includes polygons, [[15, 15], [88, 15], [88,  0], [15,  0]]
		assert_includes polygons, [[15, 74], [88, 74], [88, 60], [15, 60]]
		assert_includes polygons, [[89, 74], [99, 74], [99,  0], [89,  0]]
	end

	def test_map2
		map = img2array('test/images/map2.jpg')
		cells = BoustrophedonDecomposition.decompose(map)
		polygons = cells.map { |cell| cell.points }
		assert_equal 10, polygons.length
		assert_includes polygons, [[0, 479], [52, 479], [52, 0], [0, 0]]          
		assert_includes polygons, [[53, 479], [134, 479], [134, 297], [53, 297]]  
		assert_includes polygons, [[135, 360], [223, 360], [223, 297], [135, 297]]
		assert_includes polygons, [[135, 479], [223, 479], [223, 411], [135, 411]]
		assert_includes polygons, [[53, 165], [315, 165], [315, 0], [53, 0]]      
		assert_includes polygons, [[224, 479], [315, 479], [315, 297], [224, 297]]
		assert_includes polygons, [[316, 479], [426, 479], [426, 0], [316, 0]]    
		assert_includes polygons, [[427, 166], [597, 166], [597, 0], [427, 0]]    
		assert_includes polygons, [[427, 479], [597, 479], [597, 309], [427, 309]]
		assert_includes polygons, [[598, 479], [639, 479], [639, 0], [598, 0]]    
	end

	def test_map3
		map = img2array('test/images/map3.jpg')
		cells = BoustrophedonDecomposition.decompose(map)
		polygons = cells.map { |cell| cell.points }
		assert_equal 7, polygons.length
		assert_includes polygons, [[0, 373], [98, 373], [98, 0], [0, 0]]
		assert_includes polygons, [[266, 373], [354, 373], [354, 0], [266, 0]]
		assert_includes polygons, [[502, 373], [566, 373], [566, 0], [502, 0]]
	end

	def test_map1_start
		map = img2array('test/images/map.png')
		cells = BoustrophedonDecomposition.run(map, [20, 50])
		polygons = cells.map { |cell| cell.points }
		assert_equal 4, polygons.length
		assert_includes polygons, [[16, 58], [63, 58], [63, 17], [16, 17]]
		assert_includes polygons, [[64, 26], [72, 26], [72, 17], [64, 17]]
		assert_includes polygons, [[73, 58], [87, 58], [87, 17], [73, 17]]
		assert_includes polygons, [[64, 58], [72, 58], [72, 32], [64, 32]]
	end

	def test_map4
		map = img2array('test/images/map4.png')
		cells = BoustrophedonDecomposition.decompose(map)

		polygons = cells.map { |cell| cell.points }
		puts polygons.length
		assert_includes polygons, [[0, 36], [1, 36], [4, 39], [5, 39], [7, 41], [8, 41], [11, 44], [12, 44], [14, 46], [14, 16], [13, 17], [12, 19], [11, 20], [10, 22], [9, 23], [8, 25], [6, 27], [5, 29], [4, 30], [3, 32], [2, 33], [1, 35], [0, 36]]
		assert_includes polygons, [[17, 29], [18, 27], [18, 10], [17, 12]]
		assert_includes polygons, [[20, 27], [22, 27], [24, 25], [24, 2], [23, 3], [22, 5], [20, 7]]
		assert_includes polygons, [[25, 54], [26, 54], [28, 56], [29, 56], [32, 59], [34, 57], [35, 55], [38, 52], [39, 50], [43, 46], [44, 44], [48, 40], [49, 38], [52, 35], [53, 33], [56, 30], [56, 29], [47, 20], [46, 20], [35, 9], [34, 9], [25, 0]]

		#visualize(img, cells)
	end

	def test_jsonmap5
		file = File.open("test/json_maps/boustrophedon_request20190217-29654-kidfgo", "r")
		map = JSON.parse(file.read)

		img = Magick::Image.new(map.length, map[0].length)
		map.each_with_index do |column, x|
			column.each_with_index do |value, y|
				px = nil
				case value
					when -1
						px = Magick::Pixel.new(0xffff,0xffff,0xffff)
					else
						px = Magick::Pixel.new(0x0,0,0)
				end

				img.store_pixels(x, y, 1, 1, [px])
			end
		end
		img.write("test/images/map5.png")

		BoustrophedonDecomposition.logger.level = Logger::DEBUG
		cells = BoustrophedonDecomposition.decompose(map)
		polygons = cells.map { |cell| cell.points }
		assert_equal 1, polygons.length
		assert_includes polygons, [[0, 67], [2, 69], [3, 71], [4, 72], [5, 74], [6, 75], [7, 77], [8, 78], [9, 80], [10, 81], [11, 83], [12, 84], [13, 86], [14, 87], [15, 89], [17, 91], [18, 93], [19, 94], [20, 96], [21, 97], [22, 99], [23, 100], [24, 102], [25, 103], [26, 105], [27, 106], [28, 108], [29, 109], [30, 111], [31, 112], [32, 114], [34, 116], [35, 118], [36, 119], [37, 121], [38, 122], [39, 124], [40, 125], [41, 127], [42, 128], [43, 130], [44, 131], [45, 133], [46, 134], [47, 136], [49, 138], [50, 140], [51, 141], [52, 143], [53, 144], [54, 146], [55, 147], [56, 149], [57, 150], [58, 152], [59, 153], [60, 155], [61, 156], [62, 158], [64, 160], [65, 162], [66, 162], [68, 160], [69, 160], [71, 158], [72, 158], [74, 156], [75, 156], [78, 153], [79, 153], [81, 151], [82, 151], [84, 149], [85, 149], [87, 147], [88, 147], [90, 145], [91, 145], [94, 142], [95, 142], [97, 140], [98, 140], [100, 138], [101, 138], [103, 136], [104, 136], [106, 134], [107, 134], [109, 132], [110, 132], [113, 129], [114, 129], [116, 127], [117, 127], [119, 125], [120, 125], [122, 123], [123, 123], [125, 121], [126, 121], [129, 118], [130, 118], [132, 116], [133, 116], [135, 114], [136, 114], [138, 112], [139, 112], [141, 110], [142, 110], [145, 107], [146, 107], [148, 105], [149, 105], [151, 103], [152, 103], [154, 101], [155, 101], [157, 99], [158, 99], [160, 97], [161, 97], [162, 96], [162, 96], [161, 94], [160, 93], [159, 91], [157, 89], [156, 87], [155, 86], [154, 84], [152, 82], [151, 80], [150, 79], [149, 77], [147, 75], [146, 73], [144, 71], [143, 69], [142, 68], [141, 66], [139, 64], [138, 62], [137, 61], [136, 59], [134, 57], [133, 55], [132, 54], [131, 52], [129, 50], [128, 48], [127, 47], [126, 45], [124, 43], [123, 41], [122, 40], [121, 38], [119, 36], [118, 34], [117, 33], [116, 31], [114, 29], [113, 27], [112, 26], [111, 24], [109, 22], [108, 20], [107, 19], [106, 17], [104, 15], [103, 13], [101, 11], [100, 9], [99, 8], [98, 6], [96, 4], [95, 2], [94, 1], [92, 1], [89, 4], [88, 4], [86, 6], [85, 6], [82, 9], [81, 9], [79, 11], [78, 11], [75, 14], [74, 14], [71, 17], [70, 17], [68, 19], [67, 19], [64, 22], [63, 22], [61, 24], [60, 24], [57, 27], [56, 27], [54, 29], [53, 29], [50, 32], [49, 32], [47, 34], [46, 34], [43, 37], [42, 37], [40, 39], [39, 39], [36, 42], [35, 42], [33, 44], [32, 44], [29, 47], [28, 47], [26, 49], [25, 49], [22, 52], [21, 52], [19, 54], [18, 54], [15, 57], [14, 57], [12, 59], [11, 59], [8, 62], [7, 62], [5, 64], [4, 64], [1, 67], [0, 67]]

		#visualize(img, cells)
	end
end
