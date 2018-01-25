import { Component, OnInit } from '@angular/core';
import { HttpClient, HttpHeaders } from '@angular/common/http';
import { Http, Response, Headers, RequestOptions } from '@angular/http';
import { Observable } from 'rxjs/Observable';

import { Category } from '../category';

@Component({
    selector: 'app-datapage',
    templateUrl: './datapage.component.html',
    styleUrls: ['./datapage.component.css']
})
export class DatapageComponent implements OnInit {

    private apiUrl = 'api/';

    categories:Category[] = [];

    constructor( private http: HttpClient ) { }

    ngOnInit() {
        this.getCategories();
    }

    getCategories() {
        this.http.get(this.apiUrl+'categories').subscribe(data => {
            var catData:string[] = data['categories'];
            var lenCats:number = catData.length;
            for (var i:number = 0; i<lenCats; ++i){
                var cat:string = catData[i];

                var parent:string = cat.split('_',1)[0];

                if (parent && parent!="utcTime") {
                    var catObj = new Category(parent);

                    if (cat != parent){
                        var index = cat.indexOf('_');
                        var child = cat.substring(index+1);
                        catObj.children.push(child);
                    }

                    this.categories.push(catObj);
                }
            }
            // console.log(this.categories);
        });
    }

}
